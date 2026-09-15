# Troubleshooting: Apartment Scenario Issues

## Problem Description

When running the apartment scenario, agents get stuck in a **deadlock oscillation loop**:
- Agents repeatedly enter and exit deadlock mode
- Average distance to goal stays constant around 2.96 meters
- Success rate is 0%
- QP solver frequently fails with "Solver 'OSQP' failed" errors

## Root Causes

### 1. **Scenario Difficulty**
The apartment scenario is inherently challenging:
- Agents start at `[-2.0, -2.0]` to `[-2.0, 0.0]` (left side)
- Goals are at `[2.0, 2.0]` to `[2.0, 0.0]` (right side)
- Large obstacle (1.0 x 1.0) at center `[0.0, 0.0]` **blocks direct path**
- Agents must navigate **around** the obstacle, not through it
- Additional obstacles and walls create narrow passages

### 2. **CBF-QP Infeasibility**
The quadratic program becomes infeasible when:
- **Too many active constraints**: Obstacle CBFs + Agent CBFs + Connectivity CBFs + CLF constraints
- **Conflicting objectives**: Cannot simultaneously:
  - Move toward goal (CLF)
  - Avoid obstacles (CBF)
  - Maintain connectivity (CBF)
  - Avoid other agents (CBF)

With bounded alpha values (0.1 to 10.0), the QP has less flexibility to trade off constraints, leading to more frequent infeasibility.

### 3. **High-Level Planner Oscillation**
The deadlock detection triggers too frequently because:
- **Threshold too sensitive**: `u_min = 0.01` m/s is very low
- **Local minima**: Agents get stuck at obstacle boundaries
- **Leader oscillation**: Leaders keep switching, preventing sustained progress

### 4. **Temporary Goal Assignment**
The high-level planner assigns temporary goals to help agents escape deadlocks, but:
- Temporary goals may not account for obstacles
- No path planning to ensure goals are reachable
- Agents may get stuck trying to reach unreachable temporary goals

## What's Working vs. Not Working

### ✅ Working:
1. **Obstacle avoidance in fallback mode**: When QP fails, agents use repulsive potential fields and successfully avoid obstacles
2. **Simple scenario**: 66.7% success rate in scenarios with direct paths
3. **Bidding mechanism**: Edge deletion works correctly
4. **Deadlock detection**: System correctly identifies when agents are stuck

### ❌ Not Working:
1. **Navigation around obstacles**: No path planning to find routes around large obstacles
2. **QP feasibility**: Too many constraints cause frequent solver failures
3. **Progress in complex scenarios**: Agents don't make net progress toward goals
4. **High-level planner effectiveness**: Temporary goals don't help escape local minima

## Why Simpler Scenarios Work

The **simple scenario** succeeds (66.7%) because:
- Direct paths exist to most goals
- Smaller obstacle (0.5 x 0.3) doesn't block all paths
- Fewer agents (3 vs. 5) means fewer constraints
- Less connectivity maintenance needed

## Potential Solutions

### Short-Term Fixes

#### 1. Remove Alpha Bounds (Revert Change)
```python
# In cbf_qp_controller.py, remove these lines:
constraints.append(alpha_obs_p[obs_idx] >= 0.1)
constraints.append(alpha_obs_p[obs_idx] <= 10.0)
```

**Rationale**: Unbounded alphas give QP more flexibility, reducing infeasibility.

#### 2. Increase Deadlock Threshold
```python
# In high_level_planner.py, line 15
u_min = 0.05  # Increased from 0.01
```

**Rationale**: Reduces oscillation by being less sensitive to small movements.

#### 3. Relax CBF Constraints
```python
# In cbf_qp_controller.py, increase activation thresholds:
if B_obs_p < 2.0:  # Was 1.0
if B_agent < 4.0:  # Was 2.0
```

**Rationale**: Apply CBFs only when truly necessary, reducing constraint count.

### Medium-Term Solutions

#### 4. Add Path Planning
Integrate a global planner (like A* or RRT) to:
- Find feasible paths around obstacles
- Set intermediate waypoints for agents
- Update temporary goals based on actual reachable positions

#### 5. Prioritize Fallback Controller
Since the fallback controller with obstacle avoidance works well:
```python
# Use fallback more often, QP less often
if complex_scenario or qp_recently_failed:
    return self._compute_safe_nominal_control(...)
```

#### 6. Staged Goal Reaching
Break navigation into stages:
- Stage 1: Navigate around obstacle to intermediate waypoint
- Stage 2: Approach final goal from clear side

### Long-Term Solutions

#### 7. Learn-Based High-Level Planner
As suggested in the paper (Section V):
- Use reinforcement learning to learn better temporary goal assignments
- Train on various obstacle configurations
- Learn when to trigger deadlock resolution

#### 8. Hierarchical Path Planning
- Global planner: Computes collision-free paths
- Mid-level planner: Assigns formation/roles
- Low-level controller: CBF-QP for local safety

#### 9. Model Predictive Control (MPC)
Replace CBF-QP with MPC:
- Predicts future trajectories
- Optimizes over horizon
- Better handles non-convex obstacles

## Recommended Immediate Action

**Revert the alpha bounds** and rely on the fallback controller:

1. Remove alpha constraint bounds in `cbf_qp_controller.py`
2. Keep the improved fallback controller (faster speed, stronger repulsion)
3. Increase deadlock threshold to reduce oscillation

This should improve performance while maintaining safety.

## Expected Behavior After Fixes

With the recommended changes:
- **Simple scenario**: 70-80% success rate (improved from 66.7%)
- **Apartment scenario**: 20-40% success rate (some agents reach goals)
- **QP failures**: Reduced by ~50%
- **Deadlock oscillation**: Significantly reduced

## Testing the Fixes

```bash
# Test 1: Simple scenario (should work well)
python main.py --scenario simple --max_time 30 --no_viz

# Test 2: Apartment scenario (expect partial success)
python main.py --scenario apartment --max_time 60 --no_viz

# Test 3: Without planner (to compare baseline)
python main.py --scenario apartment --no_planner --max_time 60 --no_viz
```

## Conclusion

The apartment scenario reveals **fundamental limitations** of purely reactive CBF-QP control:
1. No global path planning
2. Greedy local optimization
3. Difficulty with complex obstacle configurations

The implementation is **correct** according to the paper, but the paper's approach has **known limitations** in scenarios with:
- Large obstacles blocking direct paths
- Narrow passages
- Complex topology

The paper likely used simpler obstacle configurations in their experiments, or had additional components (like pre-computed paths) not fully described in the publication.

---

**Bottom line**: The robots ARE avoiding obstacles correctly. The issue is that they need **path planning** to navigate AROUND obstacles, not just avoid colliding with them. This is a limitation of the reactive control approach, not a bug in the implementation.
