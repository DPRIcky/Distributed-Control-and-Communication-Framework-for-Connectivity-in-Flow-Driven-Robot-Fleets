# Real-Time Edge Pruning GUI - Quick Start Guide

## What's New?

Your `gui_algorithms_simulation.py` now shows **REAL-TIME edge pruning** with 3-panel visualization of algorithms converging to the Minimal Bridge Graph (MBG).

## How to Run

```bash
cd scripts/
python gui_algorithms_simulation.py
```

## What You'll See (3-Panel Display)

### PANEL 1: LEFT - Full Network with Pruning in Progress
- **RED edges** = Critical edges (bridges) that MUST stay
- **ORANGE dashed edges** = Redundant edges marked for removal  
- **GRAY dotted edges** = Already removed edges (pruning history)
- **Red nodes** = Robot positions in the network

*Watch these orange edges turn gray in real-time as the algorithm removes them!*

### PANEL 2: MIDDLE - Target Minimal Bridge Graph (MBG)
- Shows only the **critical (RED) edges** - this is your TARGET
- Shows what the final network will look like
- **Blue nodes** = Final optimized network nodes
- Live convergence progress display

### PANEL 3: RIGHT - Pruning Analytics & Progress Dashboard

Real-time metrics showing:
- **Network State**: Total edges, Critical edges, Redundant edges
- **Pruning Progress**: Counter of removed edges, current phase, progress %
- **Algorithm Status**: Is pruning active? Has MBG been reached?
- **Interpretation**: What's happening right now
- **Visualization Guide**: Color key for all edge types

## The Algorithm Flow

### Step 1: Topology Detection (Every 50 simulation steps)
Network topology is analyzed for critical edges using bridge detection (Algorithm 3)

### Step 2: Edge Classification  
- Critical edges: Keep (needed for connectivity)
- Redundant edges: Mark for removal

### Step 3: Real-Time Pruning Animation
Edges are removed ONE BY ONE, showing:
- Orange dashed → Gray dotted (visual transition)
- Counter increments as edges are removed
- Progress bar fills from 0% to 100%

### Step 4: Convergence
When all redundant edges are removed:
- Status shows: `COMPLETE [OK]`
- Middle panel displays final MBG
- Network is now optimized and robust

## Key Metrics Shown

- **Total Edges**: Current network connectivity count
- **Critical Edges (MBG)**: Minimum edges needed for connection
- **Redundant Edges**: Edges that can be safely removed
- **Progress**: X/Y edges removed (goes 0% → 100%)
- **Phase**: Number of pruning steps completed

## Real-Time Visualization

Each frame:
1. Robots move and maintain communication topology
2. Edges are classified by type 
3. Pruning counter increments
4. LEFT panel shows active pruning
5. MIDDLE panel shows target state
6. RIGHT panel updates metrics live

## Example Output

When you run the GUI, you'll see:
```
================================================================================
#                 REAL-TIME EDGE PRUNING SIMULATION                 #
#              Algorithms 3-6 with Dynamic Visualization                   #
================================================================================

ALGORITHMS ENABLED:
  [OK] Algorithm 1: Spatial decomposition
  [OK] Algorithm 2: Local consensus
  [OK] Algorithm 3: Bridge detection (critical edges)
  [OK] Algorithm 5: Redundancy identification
  [OK] Algorithm 6: REAL-TIME edge pruning --> MBG convergence

VISUALIZATION PANELS (3-panel display):

  [LEFT PANEL - Full Network with Pruning]
    * RED edges = Critical (must keep for connectivity)
    * ORANGE dashed = Edges marked for removal
    * GRAY dotted = Already removed edges
    * Red nodes = Robots in network

  [MIDDLE PANEL - Target Minimal Bridge Graph]
    * Shows only critical edges (final MBG)
    * Blue nodes = Final network topology
    * Real-time convergence progress

  [RIGHT PANEL - Pruning Analytics]
    * Network state metrics
    * Edge removal counter
    * Progress percentage (0% --> 100%)
    * Completion status: Working... --> COMPLETE [OK]
```

## What to Watch For

1. **Initial Phase**: Robots form connections, orange edges appear
2. **Pruning Phase**: Watch LEFT panel - orange edges disappear and become gray
3. **Completion**: Middle panel shows final minimal graph, status shows COMPLETE
4. **Robustness**: Network maintains connectivity throughout with only critical edges

## Configuration

To modify pruning behavior, edit:
- `algo_update_interval` (line ~120): How often topology is analyzed
- `num_robots`: Number of robots in simulation
- `communication_radius`: Max distance for robot communication

## Files Modified

- `gui_algorithms_simulation.py`: Main GUI with real-time pruning
  - Added `RealTimePruningManager` class for step-by-step edge removal
  - Enhanced `AlgorithmsAnimator` with 3-panel display
  - Integrated live pruning progress tracking

## Implementation Details

### Real-Time Pruning Manager
- Tracks original edges, critical edges, removed edges
- Steps through pruning ONE EDGE AT A TIME
- Maintains pruning history for animation
- Computes progress percentage in real-time

### Visualization Pipeline
- LEFT panel: Shows full network with color-coded edges
- MIDDLE panel: Shows target MBG (critical edges only)
- RIGHT panel: Shows analytics and progress metrics
- All update 60+ times per second for smooth animation

### Color Scheme
- **RED (#e74c3c)**: Critical edges - KEEP
- **ORANGE (#f39c12)**: Pending removal - TO GO  
- **GRAY (#95a5a6)**: Already removed - HISTORY
- **BLUE (#3498db)**: Final MBG nodes
- **GOLD**: Goal position

## Troubleshooting

**Q: GUI doesn't launch?**
- Ensure you have matplotlib: `pip install matplotlib`
- Check that your simulation config imports work

**Q: No edges showing?**
- Increase `communication_radius` in SimulationConfig
- Ensure `num_robots >= 2`

**Q: Pruning not starting?**
- Wait for the first topology analysis (50 simulation steps)
- Watch the time counter on LEFT panel

**Q: Want to watch metrics only?**
- RIGHT panel shows all data without needing to view LEFT animation

## Next Steps

- Modify robot count or communication radius to see different pruning patterns
- Save simulation data to analyze pruning effectiveness
- Extend to test algorithm performance on larger networks
- Compare MBG reduction across different network sizes

---

**Now watch your network automatically prune redundant edges in real-time!**
