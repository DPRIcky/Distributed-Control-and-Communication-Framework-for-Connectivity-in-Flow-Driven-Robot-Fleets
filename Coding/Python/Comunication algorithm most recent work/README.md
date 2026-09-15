# Hybrid Underwater Consensus Pruning Simulation

A multi-robot simulation demonstrating **decentralized edge pruning** in underwater environments with flow fields, using **Control Lyapunov Functions (CLF)** and **Control Barrier Functions (CBF)** for goal-seeking and safety.

---

## Overview

This simulation models a swarm of underwater robots that:
- **Navigate** through dynamic flow fields toward a goal position
- **Maintain connectivity** using a communication graph
- **Prune redundant edges** via distributed consensus to optimize network topology
- **Ensure safety** through collision avoidance and connectivity preservation

### Key Features

- **Underwater Flow Dynamics**: Robots drift in 2D current (constant drift + sinusoidal swirl)
- **Individual Robot Dynamics**: Each robot has unique mass, drag coefficient, and cross-sectional area
- **CLF-CBF Control**:
  - **CLF**: Control Lyapunov Function for goal-seeking behavior
  - **CBF**: Control Barrier Functions for collision avoidance and connectivity maintenance
- **Hybrid Consensus Pruning** with multi-layer robustness:
  - Topology stability detection
  - Rolling window validation (historical consistency)
  - Interleaved distributed consensus execution
  - Final safety checks before edge removal

---

## How It Works

### 1. **Robot Initialization**
- Robots spawn in a cluster at the left side of the workspace
- Each robot has randomized physical properties (mass, drag, area)
- A random goal position is generated on the right side
- Initial communication graph forms based on proximity (communication radius = 2.5m by default)

### 2. **Robot Motion** (Each Simulation Step)
Robots compute control forces using **CLF-CBF framework**:

- **CLF (Goal-Seeking)**: Attracts robot toward goal position
  ```
  u_clf = -k_clf * (x - x_goal)
  ```

- **CBF Safety (Collision Avoidance)**: Repels robots from each other
  ```
  If distance < 2 * safety_distance:
    u_cbf_safety = k_safety * barrier_gradient
  ```

- **CBF Connectivity**: Keeps robots connected to their parent in the connectivity tree
  ```
  If distance_to_parent > 0.95 * comm_radius:
    u_cbf_conn = k_conn * direction_to_parent
  ```

Robots then drift with flow field + control + diffusion noise.

### 3. **Hybrid Consensus Pruning**
The system uses a **multi-stage validation process** to safely prune redundant edges:

#### **Stage 1: Topology Stabilization**
- Monitor for edge formations/breaks
- Wait for `stability_threshold` (default: 5) steps without topology changes
- If topology changes, reset and restart

#### **Stage 2: Historical Validation**
- Maintain rolling window of `rolling_window_size` (default: 10) topology snapshots
- Find edges that are:
  - Currently redundant (alternative path exists)
  - Graph remains connected without the edge
  - Redundant in ≥3 out of last 5 snapshots (stability check)
- Prefer pruning longest edges (weakest connections)

#### **Stage 3: Distributed Consensus**
- Run `consensus_rounds_per_step` (default: 3) consensus iterations per simulation step
- Each robot independently evaluates edge redundancy
- Continue until consensus reached or max rounds (50) exceeded
- Track failed candidates to avoid repeated attempts

#### **Stage 4: Final Safety Check**
- Verify topology hasn't changed during consensus
- Only prune if all checks pass

### 4. **Connectivity Tree**
- Built via BFS from a root node (closest to initial cluster center)
- Used for CBF connectivity control
- Isolated robots (no path to root) are highlighted in orange

---

## Requirements

```bash
pip install numpy matplotlib
```

**Dependencies**:
- `numpy` - numerical computations
- `matplotlib` - visualization and animation
- `distributed_consensus_pruning` - custom module for consensus algorithm (must be in same directory)

---

## Usage

### Basic Syntax

```bash
python underwater_consensus_chain_hybrid.py [MODE] [OPTIONS]
```

### Modes

| Mode | Description |
|------|-------------|
| `gui` | Interactive animation with real-time visualization (default) |
| `text` | Console-based simulation with text output |

### Command-Line Options

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `--robots` | int | 8 | Number of robots (minimum: 3) |
| `--steps` | int | 100 | Simulation steps (text mode only) |
| `--quiet` | flag | False | Suppress per-step console messages |

---

## Examples

### 1. **Run GUI with Default Settings** (8 robots, interactive)
```bash
python underwater_consensus_chain_hybrid.py
# or explicitly
python underwater_consensus_chain_hybrid.py gui
```

### 2. **Run GUI with 12 Robots**
```bash
python underwater_consensus_chain_hybrid.py gui --robots 12
```

### 3. **Run Text Mode with 10 Robots for 200 Steps**
```bash
python underwater_consensus_chain_hybrid.py text --robots 10 --steps 200
```

### 4. **Run Text Mode with Minimal Output**
```bash
python underwater_consensus_chain_hybrid.py text --quiet
```

### 5. **Small Swarm Test (3 robots, GUI)**
```bash
python underwater_consensus_chain_hybrid.py gui --robots 3
```

### 6. **Large Swarm Simulation (20 robots, 500 steps, text)**
```bash
python underwater_consensus_chain_hybrid.py text --robots 20 --steps 500
```

---

## Understanding the GUI

### Visual Elements

| Element | Color | Meaning |
|---------|-------|---------|
| **★ Star** | Gold | Goal position |
| **Circles** | Red | Robots connected to tree |
| **Circles** | Orange | Robots isolated from connectivity tree |
| **Lines** | Purple | Active communication edges |
| **Lines** | Orange (dashed) | Edge being evaluated for pruning |
| **Lines** | Thick Red | Recently pruned edge (brief highlight) |
| **Arrows** | Light Blue | Flow field velocity vectors |

### Status Panel (Top Left)
Displays:
- Current time and edge counts
- Number of isolated robots
- Average distance to goal
- Robots that reached goal
- Current decision state (topology unstable, evaluating, pruned, etc.)

### Decision States

| State | Meaning |
|-------|---------|
| `TOPOLOGY UNSTABLE` | Edges broke/formed, waiting for stability |
| `Waiting for stability` | Counting stable rounds (N/5) |
| `Building history` | Accumulating topology snapshots (N/10) |
| `No persistent redundant edges` | All edges are necessary |
| `Evaluating: (i,j)` | Running consensus on edge (i,j) |
| `PRUNED (i,j)` | Successfully removed edge (i,j) |
| `FAILED: (i,j) NOT redundant` | Consensus determined edge is needed |
| `ABORTED: Topology changed` | Topology changed during consensus |

---

## Configuration Parameters

Key parameters in `HybridUnderwaterSimulation.__init__()`:

### Physical Parameters
```python
communication_radius = 3.0    # Max distance for communication (meters)
dt = 0.05                     # Simulation timestep (seconds)
workspace_size = (10.0, 10.0) # Workspace dimensions (meters)
safety_distance = 1.2         # Collision avoidance threshold (meters)
```

### Control Gains
```python
clf_gain = 0.8               # Goal-seeking strength
cbf_safety_gain = 4.0        # Collision avoidance strength
cbf_connectivity_gain = 0.5  # Parent connectivity strength
```

### Hybrid Consensus Parameters
```python
stability_threshold = 5           # Steps to wait for stable topology
rolling_window_size = 10          # Historical snapshots to maintain
consensus_rounds_per_step = 3     # Consensus iterations per step
total_consensus_rounds_needed = 50 # Max rounds before giving up
```

### Flow Field
```python
base_vector = [0.20, 0.05]   # Constant drift (m/s)
swirl_amplitude = 0.10       # Swirl perturbation amplitude
swirl_scale = 5.5            # Swirl spatial frequency
```

---

## Output Explanation

### Text Mode Output

```
Goal position: (8.23, 4.56)
Starting HYBRID simulation with 8 robots
Stability threshold: 5 steps
Rolling window: 10 snapshots
Consensus rounds per step: 3

[Step 001] t=0.05s | Decision: waiting_for_stability
[Step 006] t=0.30s | Decision: building_history
[Step 020] t=1.00s | Decision: evaluating
[Step 035] t=1.75s | Decision: prune
  ✓ Pruned: (3, 5), Edges left: 12

============================================================
Final pruned edges: [(2, 7), (3, 5), (4, 6)]
Final edge count: 9
```

### GUI Console Output

```
Goal position: (7.89, 5.12)
Starting HYBRID GUI with 8 robots
Multi-layer robustness enabled:
  - Topology stability detection
  - Rolling window validation (10 snapshots)
  - Interleaved consensus (3 rounds/step)
  - Final safety checks before pruning
```

---

## Algorithm Flow

```
┌─────────────────────────────────────────────────────┐
│ 1. Initialize: Spawn robots, build initial graph   │
└─────────────────────────────────────────────────────┘
                       ↓
┌─────────────────────────────────────────────────────┐
│ 2. Simulation Step:                                 │
│    - Build connectivity tree                        │
│    - Compute CLF-CBF control for each robot         │
│    - Update positions (flow + control + diffusion)  │
└─────────────────────────────────────────────────────┘
                       ↓
┌─────────────────────────────────────────────────────┐
│ 3. Detect Topology Changes:                         │
│    - Edges broken/formed?                           │
│      → YES: Reset stability counter, abort consensus│
│      → NO: Increment stability counter              │
└─────────────────────────────────────────────────────┘
                       ↓
┌─────────────────────────────────────────────────────┐
│ 4. Wait for Stability:                              │
│    - Stable for threshold steps? → NO: Wait         │
│                                   → YES: Continue   │
└─────────────────────────────────────────────────────┘
                       ↓
┌─────────────────────────────────────────────────────┐
│ 5. Build History:                                   │
│    - Add snapshot to rolling window                 │
│    - Enough snapshots? → NO: Wait                   │
│                        → YES: Continue              │
└─────────────────────────────────────────────────────┘
                       ↓
┌─────────────────────────────────────────────────────┐
│ 6. Find Redundant Edge:                             │
│    - Search for persistently redundant edge         │
│    - Found? → NO: Skip to step 2                    │
│            → YES: Start consensus                   │
└─────────────────────────────────────────────────────┘
                       ↓
┌─────────────────────────────────────────────────────┐
│ 7. Distributed Consensus:                           │
│    - Run consensus rounds (interleaved)             │
│    - Agreement reached? → YES: Go to step 8         │
│    - Max rounds exceeded? → YES: Blacklist edge     │
│                           → NO: Continue consensus  │
└─────────────────────────────────────────────────────┘
                       ↓
┌─────────────────────────────────────────────────────┐
│ 8. Final Safety Check:                              │
│    - Topology still unchanged?                      │
│      → YES: PRUNE EDGE, reset state                 │
│      → NO: Abort, reset state                       │
└─────────────────────────────────────────────────────┘
                       ↓
                 Return to Step 2
```

---

## Troubleshooting

### Issue: No edges being pruned
**Possible Causes**:
- Topology is too dynamic (edges constantly forming/breaking)
- All edges are truly necessary for connectivity
- Consensus rounds insufficient (increase `total_consensus_rounds_needed`)

**Solutions**:
- Increase `stability_threshold` to wait longer for stable topology
- Reduce flow field strength (lower `base_vector` magnitude)
- Increase `communication_radius` for more redundant connections

### Issue: Robots not reaching goal
**Possible Causes**:
- Flow field too strong
- CLF gain too weak
- Too many edges pruned (robots become isolated)

**Solutions**:
- Increase `clf_gain` (stronger goal attraction)
- Decrease flow `base_vector` magnitude
- Reduce `communication_radius` (fewer edges to prune)

### Issue: Robots colliding
**Possible Causes**:
- CBF safety gain too weak
- Safety distance too small

**Solutions**:
- Increase `cbf_safety_gain`
- Increase `safety_distance`

### Issue: "No module named 'distributed_consensus_pruning'"
**Cause**: Missing dependency module

**Solution**:
Ensure `distributed_consensus_pruning.py` is in the same directory as this script.

---

## Research Context

This simulation demonstrates:
- **Decentralized control** in multi-robot systems
- **Safety-critical control** using CBFs
- **Distributed consensus algorithms** for network optimization
- **Hybrid decision-making** combining reactive and deliberative layers
- **Robustness** to dynamic environments and topology changes

**Applications**:
- Underwater exploration and surveying
- Oceanic monitoring networks
- Autonomous underwater vehicle (AUV) swarms
- Resilient communication networks

---

## License

Research code for academic purposes.

---

## Contact

For questions or issues, refer to the source code comments or contact the research team.
