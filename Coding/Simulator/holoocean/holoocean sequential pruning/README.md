# HoloOcean Sequential Pruning Workspace

This workspace contains research and implementation of distributed spanning-tree pruning algorithms for multi-agent systems, specifically designed for integration with the Holoocean underwater robotics simulator.

## Overview

The focus of this workspace is on developing communication-efficient algorithms for maintaining network topology in multi-agent underwater systems. The key innovation is a distributed pruning algorithm that uses only O(1) communication (scalar hop distance propagation) rather than O(n) distance vectors, making it feasible for bandwidth-constrained underwater acoustic communication.

## Key Components

### Core Algorithm
- **`distributed_pruning_algorithm.py`**: Implementation of the Distributed Spanning-Tree Pruning Algorithm (Comm-Feasible Refactor)
  - Phase 1: δ-BFS - Distributed Bellman-Ford for hop distance computation
  - Phase 3: Spanning-tree construction using parent pointers
  - Removed communication-infeasible phases (Connectivity Assurance and Robustness Enhancement)
  - Features: configurable packet drop probability, random delay, asynchronous broadcasting

### Related Projects
This workspace is part of a larger pruning research effort:
- **`../holoocean_pruning_project/`**: Structured Python package with the same algorithm
  - Installable via `pip install -e .`
  - Includes test scripts and integration examples
  - Organized as a proper Python package (`pruning/` directory)

## Phase 2 Updates: Internal State for Sequential Pruning

In this phase, we extended the internal per-node state in `distributed_pruning_algorithm.py` to support future sequential pruning while preserving the current delta-BFS algorithm behavior.

### New Internal State Fields

The following fields were added to the `NodeState` dataclass:

- `committed_parent`: The parent that is currently committed (stable) for sequential pruning.
- `backup_parent`: A backup parent for fast failover.
- `edge_status`: Dictionary mapping neighbor ID to edge status string (e.g., `EDGE_STATUS_PARENT`, `EDGE_STATUS_BACKUP`, `EDGE_STATUS_GRACE`, `EDGE_STATUS_DELETE_CANDIDATE`, `EDGE_STATUS_UNUSED`).
- `delete_counter`: Dictionary mapping neighbor ID to delete counter (used for graceful edge removal).
- `grace_counter`: Dictionary mapping neighbor ID to grace counter (used for hysteresis in edge retention).
- `last_heard_time`: Dictionary mapping neighbor ID to last heard simulation time (for freshness tracking).

Note: The existing `parent` field continues to represent the candidate parent from the delta-BFS algorithm, preserving current behavior.

### Distinction Between Candidate and Committed Topology

- **Candidate Topology**: The topology suggested by the latest delta-BFS computation (represented by the `parent` field). This may change frequently as new delta information arrives.
- **Committed Topology**: The topology that is stable and used for control (represented by the `committed_parent` field). Changes to the committed topology are subject to hysteresis and grace periods to prevent flapping.

In this phase, the committed topology is not yet used; the algorithm still uses the candidate topology for parent selection and tree construction. The new state fields are initialized and reset appropriately but do not yet influence the algorithm.

### What Phase 2 Changed

- Extended `NodeState` with sequential pruning state fields.
- Initialized per-neighbor state in the `__init__` method.
- Added state resetting in `reset_epoch_state`.
- Preserved all existing delta-BFS algorithm logic and communication-feasible design.

### What Remains for Later Phases

- Implement the sequential pruning policy in the `_on_receive` method to update `committed_parent` based on hysteresis and grace periods.
- Use the committed topology for tree construction and pruning commitment.
- Potentially adjust the `build_spanning_tree` method to use committed parents.
- Tune sequential pruning parameters (e.g., grace period duration, hysteresis threshold).

## Phase 3 Updates: Sequential Pruning Logic Activated

In this phase, we activated the sequential pruning logic by implementing the policy that uses the internal state added in Phase 2. The algorithm now distinguishes between candidate and committed topology, selects backup parents, classifies edge states, and applies grace periods and delayed deletion to reduce parent flapping and improve stability.

### What Phase 3 Changed

- **Candidate vs. Committed Topology**: The `parent` field remains the candidate parent from delta-BFS. The `committed_parent` field now represents the stable parent used for the tree, updated only after a grace period to prevent flapping.
- **Backup Parent Selection**: When enabled, each node selects a backup parent (the neighbor with the best delta_to_root that is not the committed parent) for fast failover. Neighbors with no path to the root (infinite delta_to_root) are excluded from consideration.
- **Edge-State Classification**: For each incident edge, the node locally classifies the edge into one of five states: `PARENT`, `BACKUP`, `GRACE`, `DELETE_CANDIDATE`, or `UNUSED`, based on the candidate/committed parents, backup parent, and timers.
- **Delayed Deletion**: Edges in `DELETE_CANDIDATE` state have a delete timer that starts when the edge enters this state. The edge remains in the maintained topology (kept in `kept_neighbors`) during the delete timer period. Only when the elapsed time exceeds the threshold `H` (configurable) does the edge transition to `UNUSED` state and get removed from the maintained topology.
- **Grace Period Retention**: When a node’s candidate parent changes, the old committed parent edge is retained in `GRACE` state for a configurable period `G`. During this grace period, the old edge is kept in the maintained topology and the committed parent does NOT change (preventing flapping). Only after the grace period expires does the committed parent update to the new candidate parent.
- **Helper Methods**: Added modular helper methods for edge status classification, backup parent selection, committed parent update, and sequential state evolution.
- **Configurable Parameters**: Added `delete_threshold_h` (default 3.0 seconds) and `grace_period_g` (default 2.0 seconds) to control the timing thresholds, and `enable_backup` (default True) to toggle backup parent selection.

### Distinction Between Candidate and Committed Topology

- **Candidate Topology**: The topology suggested by the latest delta-BFS computation (represented by the `parent` field). This may change frequently as new delta information arrives.
- **Committed Topology**: The topology that is stable and used for control (represented by the `committed_parent` field). Changes to the committed topology are subject to hysteresis and grace periods to prevent flapping.

The algorithm now uses the committed topology for tree construction (via `build_spanning_tree`), while the candidate topology is used only for detecting potential parent changes.

### What Remains for Later Phases

- Tune the sequential pruning parameters (grace period duration, hysteresis threshold, delete threshold) for specific deployment scenarios.
- Evaluate the performance under various loss and delay models.
- Integrate with HoloOcean (future phase).

## Communication-Feasible Design

The algorithm is designed for realistic underwater communication constraints:

1. **O(1) Message Payload**: Each broadcast contains only `(agent_id, delta_to_root, seq_number)` = ~6 bytes
2. **Configurable Loss Models**:
   - Packet drop probability (`p_drop`)
   - Random propagation delay (`delay_max`)
   - Gilbert-Elliott burst loss model available
3. **Asynchronous Operation**: Jitter prevents transmission collisions
4. **Bandwidth Accounting**: Tracks TX-attempted vs RX-delivered metrics

## HoloOcean Integration Notes

The algorithm is designed to map cleanly into HoloOcean:

1. **Agent Broadcast**: Each agent periodically broadcasts its hop distance to root
2. **Network Layer**: Holoocean's network model applies packet loss and delay
3. **Epoch Structure**:
   - Topology update epochs run for `t_prune` seconds
   - After each epoch, agents maintain edges to their parents (pruned topology)
   - Pruned topology feeds into CLF-CBF controllers for formation control
4. **Periodic Updates**: Epochs can run periodically or on topology change detection

## File Structure

```
holoocean sequential pruning/
├── README.md                 ← This file
├── distributed_pruning_algorithm.py  ← Main algorithm implementation
├── verify_default.py         ← Default parameter verification
├── verify_phase2.py          ← Phase 2 (internal state) verification
├── verify_phase3.py          ← Phase 3 (sequential pruning logic) verification
├── verify_grace_deterministic.py   ← Deterministic grace period verification
├── verify_delete_candidate_deterministic.py  ← Deterministic DELETE_CANDIDATE verification
└── demonstrate_fixes.py      ← Demonstration of fixed behaviors
```

## Related Documentation

See the notes directory in the parent pruning project for iteration logs:
- `../holoocean_pruning_project/notes/iter_*.md` - Development iteration notes
- `../holoocean_pruning_project/notes/grace_period_sim_notes.md` - Detailed simulation notes

## Dependencies

- Python 3.7+
- Holoocean simulator (for full integration)
- Standard Python libraries (no external dependencies for the core algorithm)

## Usage

To run the algorithm in isolation:
```python
from distributed_pruning_algorithm import DistributedPruningAlgorithm
# Or: from pruning.distributed_pruning_algorithm import DistributedPruningAlgorithm
```

For full Holoocean integration, see the example scripts in:
`../holoocean_pruning_project/holoocean_runs/`

## Research Context

This work addresses the challenge of maintaining reliable communication topologies in underwater multi-agent systems where:
- Bandwidth is severely limited (acoustic communication)
- Packet loss is significant and often bursty
- Propagation delays are large and variable
- Traditional O(n²) or O(n) communication algorithms are infeasible

The communication-feasible approach enables scalable multi-agent coordination under realistic underwater constraints.

## Current Status / Ready for Next Stage

This workspace now contains a fully functional standalone sequential-pruning extension of the delta-BFS algorithm with the following characteristics:
- O(1) communication payload (scalar hop distance propagation)
- Distinguishes between candidate topology (from delta-BFS) and committed topology (stable parent selection)
- Implements backup parent selection for fast failover
- Applies grace-period retention to prevent parent flapping
- Implements delayed deletion to allow smooth edge removal
- Uses committed topology for tree construction (via `build_spanning_tree`)
- Node-local update logic remains one-hop and lightweight
- Deterministic verification scripts confirm the correctness of grace period and DELETE_CANDIDATE state machines

The algorithm is now ready for the next stage: integration with the HoloOcean runner (see `../holoocean_pruning_project/holoocean_runs/` for example integration scripts).

## License

See the main Holoocean repository LICENSE file for licensing information.