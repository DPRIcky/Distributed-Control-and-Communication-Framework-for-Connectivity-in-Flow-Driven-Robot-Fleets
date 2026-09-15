"""Headless consensus-based adjacency pruning engine.

This module re-implements the ``ConsensusProofSimulation.step()`` loop from
``examples/gui_simulation_consensus.py`` **without** any matplotlib / GUI
dependency so it can be imported in CI, batch scripts, or unit tests.

The pruning semantics are preserved exactly:

1. Stability window – topology must be unchanged for ``stability_threshold``
   consecutive steps before consensus starts.
2. Adjacency-matrix consensus – iterated until the max pair-wise disagreement
   drops below ``consensus_convergence_epsilon`` (or 50 iterations).
3. Prune-only-if-all-agree – every robot must independently nominate the same
   redundant edge before it is actually removed.

New addition: **message accounting**.  Every consensus iteration and every
decision-broadcast round is tracked so that the returned metrics include
``tx_messages``, ``rx_messages``, ``tx_bytes``, ``rx_bytes``, and
``bytes_per_second``.

Neighbor-list payload model
---------------------------
Each broadcast from robot *i* carries its current neighbor-list (a set of
``uint32`` robot IDs).  Payload = ``|N_i| × 4`` bytes.  In one consensus
round every robot sends this payload to each of its neighbors, giving:

    tx_messages_per_round  =  Σ_i |N_i|  =  2·|E|
    tx_bytes_per_round     =  Σ_i |N_i|² · 4

RX aggregates are identical (every sent byte is received in an undirected
graph).
"""

from __future__ import annotations

import sys
import time as _walltime
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence

import numpy as np

# ---------------------------------------------------------------------------
# Ensure the workspace root is on sys.path so "config", "simulation", … can
# be resolved when consensus_baseline is installed editable or run from the
# workspace.
# ---------------------------------------------------------------------------
_WORKSPACE_ROOT = str(Path(__file__).resolve().parent.parent)
if _WORKSPACE_ROOT not in sys.path:
    sys.path.insert(0, _WORKSPACE_ROOT)

from config import SimulationConfig, ControlConfig, ConsensusConfig
from simulation import HybridUnderwaterSimulation

# ---------------------------------------------------------------------------
# Constants for message-size accounting
# ---------------------------------------------------------------------------
BYTES_PER_NEIGHBOR_ID: int = 4       # one uint32 per neighbor
BYTES_PER_DECISION_MSG: int = 8      # edge tuple  (2 × uint32)


# ═══════════════════════════════════════════════════════════════════════════
# Headless simulation
# ═══════════════════════════════════════════════════════════════════════════
class HeadlessConsensusSimulation(HybridUnderwaterSimulation):
    """Non-GUI consensus pruning simulation with message accounting.

    This is a drop-in headless replacement for ``ConsensusProofSimulation``
    from ``gui_simulation_consensus.py``.  All matplotlib / animation code
    has been stripped; the pruning + consensus logic in ``step()`` is kept
    intact.
    """

    def __init__(
        self,
        sim_config: SimulationConfig,
        control_config: ControlConfig,
        consensus_config: ConsensusConfig,
    ) -> None:
        super().__init__(sim_config, control_config, consensus_config)
        self.using_novel_method: bool = True
        self.consensus_phase_done: bool = False

        # ── message accounting ──────────────────────────────────────────
        self.tx_messages: int = 0
        self.rx_messages: int = 0
        self.tx_bytes: int = 0
        self.rx_bytes: int = 0

        # ── pruning bookkeeping ─────────────────────────────────────────
        self.pruning_events: List[Dict[str, Any]] = []

    # -----------------------------------------------------------------
    # Message accounting helpers
    # -----------------------------------------------------------------
    def _account_consensus_round(self) -> None:
        """Account for one round of neighbor-list broadcasts (consensus).

        Every robot *i* sends its neighbor list to each of its ``|N_i|``
        neighbors.  Payload per message = ``|N_i| × BYTES_PER_NEIGHBOR_ID``.
        """
        for neighbors in self.topology_manager.neighbor_graph:
            n = len(neighbors)
            if n == 0:
                continue
            payload = n * BYTES_PER_NEIGHBOR_ID
            self.tx_messages += n
            self.tx_bytes += n * payload
            self.rx_messages += n
            self.rx_bytes += n * payload

    def _account_decision_broadcast(self) -> None:
        """Account for one round of decision broadcasts.

        Every robot broadcasts its pruning decision (8-byte edge tuple or
        nothing) to each neighbor.
        """
        for neighbors in self.topology_manager.neighbor_graph:
            n = len(neighbors)
            if n == 0:
                continue
            self.tx_messages += n
            self.tx_bytes += n * BYTES_PER_DECISION_MSG
            self.rx_messages += n
            self.rx_bytes += n * BYTES_PER_DECISION_MSG

    # -----------------------------------------------------------------
    # Core step – mirrors ConsensusProofSimulation.step() verbatim
    # -----------------------------------------------------------------
    def step(self) -> dict:  # noqa: C901  (complexity from original)
        """Execute one simulation step with consensus-based pruning.

        Preserves the full pruning semantics:
        * stability window
        * adjacency-matrix consensus (up to 50 iterations)
        * prune-only-if-all-agree
        """
        # STEP 1: Update connectivity tree
        self.tree_builder.build_tree(
            self.robots, self.topology_manager.neighbor_graph
        )

        # STEP 2: Move robots (CLF-CBF control)
        for robot_id, robot in enumerate(self.robots):
            control_force = self.controller.compute_control(
                robot_id, self.robots
            )
            robot.step(self.flow, self.dt, self.time, self.workspace, control_force)

        self.time += self.dt

        # STEP 3: Detect topology changes
        old_edges = set(self.current_edges())
        self.topology_manager.update_neighbor_graph(self.robots)
        new_edges = set(self.current_edges())

        edges_broken = old_edges - new_edges
        edges_formed = new_edges - old_edges
        topology_changed = len(edges_broken) > 0 or len(edges_formed) > 0

        # STEP 4: Handle topology changes
        if topology_changed:
            self.pruning_manager.reset_on_topology_change()
            self.consensus_phase_done = False
            if self.verbose:
                print(
                    f"[t={self.time:5.2f}] Topology changed! Resetting consensus"
                )
            return {
                "decision": "topology_unstable",
                "edges_broken": sorted(edges_broken),
                "edges_formed": sorted(edges_formed),
            }
        else:
            self.pruning_manager.increment_stability()

        # STEP 5: Wait for stability
        if not self.pruning_manager.is_stable():
            return {
                "decision": "waiting_for_stability",
                "stable_rounds": self.pruning_manager.topology_stable_rounds,
                "threshold": self.consensus_config.stability_threshold,
            }

        # STEP 6: Rolling-window snapshot
        self.pruning_manager.add_topology_snapshot(
            new_edges,
            self.topology_manager.gather_edge_lengths(self.robots),
            self.time,
        )

        # STEP 7: Need enough history
        if not self.pruning_manager.has_sufficient_history():
            return {
                "decision": "building_history",
                "history_size": len(self.pruning_manager.topology_history),
                "needed": self.consensus_config.rolling_window_size,
            }

        # ── NOVEL CONSENSUS-BASED PRUNING ──────────────────────────────
        positions = np.array([r.position for r in self.robots], dtype=float)
        edge_lengths = self.topology_manager.gather_edge_lengths(self.robots)

        # Run consensus phase until converged (up to 50 iterations)
        if not self.consensus_phase_done:
            if self.verbose:
                print("[NOVEL] Running adjacency-matrix consensus …")

            for iteration in range(50):
                converged = self.pruning_manager.run_consensus_phase(
                    positions=positions,
                    edge_lengths=edge_lengths,
                    current_edges=new_edges,
                )
                # ── account every consensus iteration ───────────────────
                self._account_consensus_round()

                if self.verbose and iteration % 10 == 0:
                    A0 = self.pruning_manager.get_consensus_estimate(0)
                    A1 = self.pruning_manager.get_consensus_estimate(1)
                    disagreement = np.max(np.abs(A0 - A1))
                    print(
                        f"  Iter {iteration}: disagreement = {disagreement:.6f}"
                    )

                if converged:
                    if self.verbose:
                        print(
                            f"  Consensus CONVERGED in {iteration + 1} "
                            f"iterations"
                        )
                    self.consensus_phase_done = True
                    break
            else:
                # Force convergence after max iterations
                self.pruning_manager.consensus_converged = True
                self.consensus_phase_done = True
                if self.verbose:
                    print(
                        f"  Forced convergence after {iteration + 1} iterations"
                    )

            return {"decision": "running_consensus"}

        # ── Distributed edge detection ─────────────────────────────────
        if self.verbose:
            print("[NOVEL] Using distributed edge detection …")

        robot_decisions: list = []
        for robot_id in range(self.num_robots):
            decision = self.pruning_manager.find_redundant_edge_distributed(
                robot_id=robot_id,
                debug=(self.verbose and robot_id == 0),
            )
            robot_decisions.append(decision)

        # ── account decision broadcast ─────────────────────────────────
        self._account_decision_broadcast()

        unique_decisions = set(robot_decisions)

        if len(unique_decisions) == 1 and robot_decisions[0] is not None:
            candidate = robot_decisions[0]
            if self.verbose:
                print(
                    f"[NOVEL] ALL {self.num_robots} robots AGREE: "
                    f"remove {candidate}"
                )

            # Prune the agreed-upon edge
            self.topology_manager.prune_edge(candidate, self.robots)

            # Update every robot's local estimate so the removed edge is 0
            i, j = candidate
            for rid in range(self.num_robots):
                A = self.pruning_manager.adjacency_consensus.A_estimates[rid]
                A[i, j] = 0.0
                A[j, i] = 0.0

            self.pruning_manager.reset_after_successful_prune()
            self.consensus_phase_done = False

            self.pruning_events.append(
                {
                    "time": self.time,
                    "edge": candidate,
                    "edges_remaining": len(self.current_edges()),
                }
            )

            if self.verbose:
                print(
                    f"[t={self.time:5.2f}] PRUNED {candidate} | "
                    f"edges left={len(self.current_edges())}"
                )

            return {
                "decision": "prune",
                "removed_edge": candidate,
                "method": "NOVEL_CONSENSUS",
            }
        else:
            # Robots disagree → re-run consensus
            self.consensus_phase_done = False
            if self.verbose and len(unique_decisions) > 1:
                print(f"[NOVEL] Robots disagree: {unique_decisions}")
            return {"decision": "no_consensus"}


# ═══════════════════════════════════════════════════════════════════════════
# Public API
# ═══════════════════════════════════════════════════════════════════════════
def run_adjacency_consensus_trial(
    *,
    num_robots: int = 15,
    communication_radius: float = 3.0,
    max_steps: int = 2000,
    dt: float = 0.05,
    seed: Optional[int] = None,
    verbose: bool = False,
    stability_threshold: int = 5,
    rolling_window_size: int = 10,
    lambda2_threshold: float = 0.1,
    workspace_size: tuple = (10.0, 10.0),
    **consensus_overrides: Any,
) -> Dict[str, Any]:
    """Run a single headless consensus-pruning trial and return metrics.

    Parameters
    ----------
    num_robots : int
        Number of robots in the swarm.
    communication_radius : float
        Communication range (metres).
    max_steps : int
        Maximum simulation steps.
    dt : float
        Integrator time-step (seconds).
    seed : int | None
        RNG seed for reproducibility.
    verbose : bool
        Print step-by-step diagnostics.
    stability_threshold, rolling_window_size, lambda2_threshold
        Consensus-config knobs (forwarded directly to ``ConsensusConfig``).
    workspace_size : tuple
        2-D workspace dimensions.
    **consensus_overrides
        Any remaining kwargs are set on ``ConsensusConfig``.

    Returns
    -------
    dict
        ``initial_edges``       – edge count at t = 0
        ``final_edges``         – edge count at end
        ``total_prunes``        – number of edges removed
        ``pruning_events``      – list of per-prune dicts
        ``final_lambda2``       – algebraic connectivity of final graph
        ``sim_time``            – simulation clock (seconds)
        ``wall_time``           – wall-clock duration (seconds)
        ``steps_executed``      – number of step() calls
        ``tx_messages``         – total messages transmitted
        ``rx_messages``         – total messages received
        ``tx_bytes``            – total bytes transmitted
        ``rx_bytes``            – total bytes received
        ``bytes_per_second``    – tx_bytes / sim_time
    """
    sim_cfg = SimulationConfig(
        num_robots=num_robots,
        communication_radius=communication_radius,
        dt=dt,
        workspace_size=workspace_size,
        seed=seed,
        verbose=verbose,
    )
    ctrl_cfg = ControlConfig()

    cons_kwargs: Dict[str, Any] = dict(
        stability_threshold=stability_threshold,
        rolling_window_size=rolling_window_size,
        lambda2_threshold=lambda2_threshold,
    )
    cons_kwargs.update(consensus_overrides)
    cons_cfg = ConsensusConfig(**cons_kwargs)

    sim = HeadlessConsensusSimulation(sim_cfg, ctrl_cfg, cons_cfg)

    initial_edges = len(sim.current_edges())

    wall_t0 = _walltime.perf_counter()

    for step_idx in range(max_steps):
        result = sim.step()
        # Optionally break early if no edges remain to prune
        if result.get("decision") == "no_consensus":
            # No further pruning feasible for now – but keep simulating
            pass

    wall_elapsed = _walltime.perf_counter() - wall_t0

    # ── Compute final algebraic connectivity (λ₂) ─────────────────────
    A_final = np.zeros((num_robots, num_robots))
    for i, nbrs in enumerate(sim.topology_manager.neighbor_graph):
        for j in nbrs:
            A_final[i, j] = 1.0
    D = np.diag(A_final.sum(axis=1))
    L = D - A_final
    eigvals = np.sort(np.linalg.eigvalsh(L))
    final_lambda2 = float(eigvals[1]) if len(eigvals) > 1 else 0.0

    sim_time = sim.time
    return {
        "initial_edges": initial_edges,
        "final_edges": len(sim.current_edges()),
        "total_prunes": len(sim.pruning_events),
        "pruning_events": sim.pruning_events,
        "final_lambda2": final_lambda2,
        "sim_time": sim_time,
        "wall_time": wall_elapsed,
        "steps_executed": step_idx + 1,
        "tx_messages": sim.tx_messages,
        "rx_messages": sim.rx_messages,
        "tx_bytes": sim.tx_bytes,
        "rx_bytes": sim.rx_bytes,
        "bytes_per_second": sim.tx_bytes / sim_time if sim_time > 0 else 0.0,
    }


def run_batch(
    trials: int = 10,
    *,
    seeds: Optional[Sequence[int]] = None,
    **trial_kwargs: Any,
) -> List[Dict[str, Any]]:
    """Run multiple trials and return a list of metric dicts.

    Parameters
    ----------
    trials : int
        Number of independent trials.
    seeds : sequence of int, optional
        Per-trial seeds.  If *None*, seeds ``0 … trials-1`` are used.
    **trial_kwargs
        Forwarded to :func:`run_adjacency_consensus_trial`.

    Returns
    -------
    list[dict]
        One metrics dictionary per trial.
    """
    if seeds is None:
        seeds = list(range(trials))
    if len(seeds) < trials:
        raise ValueError(
            f"Provided {len(seeds)} seeds but requested {trials} trials"
        )

    results: List[Dict[str, Any]] = []
    for idx in range(trials):
        print(f"── Trial {idx + 1}/{trials}  (seed={seeds[idx]}) ──")
        m = run_adjacency_consensus_trial(seed=seeds[idx], **trial_kwargs)
        results.append(m)
        print(
            f"   prunes={m['total_prunes']}  "
            f"λ₂={m['final_lambda2']:.4f}  "
            f"TX={m['tx_messages']}  "
            f"Bps={m['bytes_per_second']:.0f}"
        )
    return results
