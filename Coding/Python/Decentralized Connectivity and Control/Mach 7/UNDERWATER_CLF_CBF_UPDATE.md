# Underwater Consensus with CLF-CBF Goal-Seeking Control

## Overview

The `underwater_consensus_chain.py` simulation has been updated to implement **goal-seeking behavior with Control Lyapunov Functions (CLF) and Control Barrier Functions (CBF)** while robots drift in an underwater flow field.

## Key Features Added

### 1. **Goal-Seeking Behavior (CLF)**
- Each robot has a goal position (default: `[7.5, 5.0]` in a 10×10 workspace)
- Control Lyapunov Function drives robots toward the goal
- **CLF formula**: `u_CLF = -k_CLF * (x - x_goal)`
- Configurable gain parameter: `clf_gain` (default: 1.5)

### 2. **Safety Constraints (CBF)**
- **Collision Avoidance**: Robots maintain safe distances from each other
- **Barrier function**: `h_safety(x) = ||x_i - x_j||² - d_safe²`
- When `h_safety < threshold`, repulsive control is applied
- Safety distance: `0.3` m (configurable)
- CBF safety gain: `3.0` (default)

### 3. **Connectivity Constraints (CBF)**
- Robots maintain connection to their parent in the communication tree
- **Barrier function**: `h_conn(x) = R_comm² - ||x_i - x_parent||²`
- Prevents robots from drifting out of communication range
- Communication radius: `2.2` m with safety margin of 85%
- CBF connectivity gain: `2.0` (default)

### 4. **Individual Flow Effects**
Each robot experiences:
- **Flow field advection**: Currents and vortices
- **Brownian diffusion**: Random environmental disturbances
- **Velocity damping**: Water resistance (85% per step)
- **Physical properties**: Individual mass, drag coefficient, cross-sectional area

### 5. **Connectivity Tree Structure**
- Robots form a spanning tree rooted near the starting position
- Parent-child relationships maintained for connectivity
- BFS algorithm rebuilds tree each step as topology changes

## Implementation Details

### Modified Classes

#### `ChainRobot`
Added attributes:
```python
mass: float                      # Robot mass (kg)
drag_coefficient: float          # Drag coefficient
cross_sectional_area: float      # Frontal area (m²)
velocity: np.ndarray             # Current velocity state
parent: Optional[int]            # Parent robot ID in tree
volume: float                    # Robot volume for buoyancy
```

Updated `step()` method:
- Now accepts `control_force` parameter from CLF-CBF controller
- Integrates control with flow and diffusion
- Applies velocity damping

#### `UnderwaterChainSimulation`
Added parameters:
```python
goal_position: np.ndarray        # Goal location
safety_distance: float           # Minimum safe distance
clf_gain: float                  # CLF strength
cbf_safety_gain: float          # Safety CBF strength
cbf_connectivity_gain: float    # Connectivity CBF strength
```

New methods:
- `_build_connectivity_tree()`: Constructs parent-child relationships
- `_compute_clf_cbf_control(robot_id)`: Main control computation

### Control Law

The complete control for robot `i` is:

```
u_i = u_CLF + Σ u_safety_j + u_connectivity
```

Where:
1. **Goal attraction (CLF)**:
   ```
   u_CLF = -k_CLF * (x_i - x_goal)
   ```

2. **Collision avoidance (Safety CBF)**:
   ```
   if ||x_i - x_j|| < 2*d_safe:
       u_safety_j = k_safety * (-h_safety) * (x_i - x_j)/||x_i - x_j||
   ```

3. **Connectivity maintenance (Connectivity CBF)**:
   ```
   if ||x_i - x_parent|| > 0.85*R_comm:
       u_conn = k_conn * (-h_conn) * (x_parent - x_i)/||x_parent - x_i||
   ```

Control is magnitude-limited to `max_control = 2.0`

## Visualization Enhancements

### Added to GUI:
- **Gold star** marking goal position
- **Dashed circle** showing goal detection radius (0.5 m)
- **Status panel** displaying:
  - Time
  - Number of active/pruned edges
  - Average distance to goal
  - Number of robots at goal
  - Current pruning decision

## Usage

### Text Mode
```bash
python underwater_consensus_chain.py --robots 8 --steps 50
```

### GUI Mode
```bash
python underwater_consensus_chain.py gui --robots 8
```

### Custom Parameters
You can modify parameters in the code:
```python
sim = UnderwaterChainSimulation(
    num_robots=10,
    goal_position=np.array([8.0, 6.0]),  # Custom goal
    clf_gain=2.0,                        # Stronger goal attraction
    cbf_safety_gain=4.0,                 # More aggressive safety
    cbf_connectivity_gain=1.5,           # Relaxed connectivity
    safety_distance=0.4,                 # Larger safety zone
    communication_radius=2.5             # Larger comm range
)
```

## Expected Behavior

### Phase 1: Initialization (t = 0-2s)
- Robots clustered at starting position `[2.5, 5.0]`
- Full connectivity (dense network)
- Beginning to drift in flow

### Phase 2: Goal Approach (t = 2-15s)
- CLF drives robots toward goal `[7.5, 5.0]`
- CBF safety prevents collisions during motion
- CBF connectivity maintains tree structure
- Consensus pruning removes redundant edges

### Phase 3: Goal Region (t > 15s)
- Robots approach goal area
- CLF weakens near goal (distance < 0.1)
- Safety and connectivity constraints dominate
- Network settles to minimal spanning tree

## Key Metrics

Monitor these during simulation:
1. **Average distance to goal**: Should decrease over time
2. **Robots at goal**: Count within 0.5 m of goal
3. **Edge count**: Decreases as consensus prunes redundant edges
4. **Connectivity**: Maintained throughout (robots stay connected)

## Parameter Tuning Guide

### For faster goal approach:
- Increase `clf_gain` (e.g., 2.0-3.0)
- Decrease `cbf_connectivity_gain` (e.g., 1.0-1.5)

### For tighter formation:
- Increase `cbf_connectivity_gain` (e.g., 3.0-4.0)
- Decrease `safety_distance` (e.g., 0.2-0.25)

### For more flow influence:
- Increase `FlowField.swirl_amplitude` in code
- Increase `diffusion` parameter in `ChainRobot`

### For more stability:
- Increase `cbf_safety_gain` (e.g., 4.0-6.0)
- Decrease `clf_gain` (e.g., 1.0-1.2)

## Comparison with Base Code

### From `Base_decentralized_connectivity_with_underwater_flow_field_updated.py`:
- ✅ CLF-CBF control framework
- ✅ Goal-seeking behavior
- ✅ Safety and connectivity barriers
- ✅ Individual robot dynamics

### Added to `underwater_consensus_chain.py`:
- ✅ Decentralized consensus pruning
- ✅ Dynamic edge removal
- ✅ Flow field integration
- ✅ Brownian motion effects
- ✅ Real-time topology adaptation

## Future Enhancements

Potential improvements:
1. **Multiple goals**: Assign different goals to robot subgroups
2. **Adaptive gains**: Adjust CLF/CBF gains based on proximity to goal
3. **Formation control**: Maintain specific geometric patterns at goal
4. **Obstacle avoidance**: Add static/dynamic obstacles
5. **Energy optimization**: Minimize control effort while reaching goal
6. **Leader-follower**: Designate leader robots for coordinated motion

## References

- Control Lyapunov Functions (CLF) for stability
- Control Barrier Functions (CBF) for safety
- Consensus-based graph pruning for minimal spanning trees
- Underwater robotics flow field modeling

## Contact

For questions or issues, refer to the research group documentation or contact the development team.

---
**Last Updated**: October 31, 2025
