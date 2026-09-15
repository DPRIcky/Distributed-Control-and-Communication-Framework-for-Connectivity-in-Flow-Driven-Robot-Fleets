# Quick Start Guide

## Installation Verified ✓

The implementation is ready to use! All 7 installation tests passed.

## Step 1: Install Dependencies (if not already done)

```bash
pip install numpy scipy cvxpy matplotlib networkx
```

Or use the requirements file:
```bash
pip install -r requirements.txt
```

## Step 2: Run Your First Simulation

### Simple Test (3 agents, ~20 seconds)
```bash
python main.py --scenario simple --max_time 20
```

### Apartment Scenario (5 agents, Figure 3 from paper)
```bash
python main.py --scenario apartment
```

### See Deadlock Without Hierarchical Control
```bash
python main.py --scenario apartment --no_planner
```
This shows how the system gets stuck without the high-level planner!

## What You'll See

A real-time visualization window will open showing:
- **Blue circles** = Agents
- **Red circles** = Leader agents (when resolving deadlocks)
- **Green stars** = Goal locations
- **Gray lines** = Network connectivity
- **Black shapes** = Obstacles
- **Arrows** = Agent orientations

## Example Output

```
Starting simulation with 5 agents...
High-level planner: Enabled
Bidding mechanism: Enabled

t=0.00s | Avg dist to goal: 5.657 | Avg speed: 0.234
t=5.00s | Avg dist to goal: 4.123 | Avg speed: 0.189
Deadlock detected! Average speed: 0.008 < 0.01
Leader assigned: Agent 2
Agent 1 follows Agent 2
...
All agents reached their goals at t=28.50s!

==========================================================
SIMULATION COMPLETE
==========================================================
Success rate: 100.0%
```

## More Scenarios

### Crossing (10 agents, two groups)
```bash
python main.py --scenario crossing --max_time 50
```

### Narrow Passage (24 agents, most challenging)
```bash
python main.py --scenario narrow --max_time 60
```

### Run Without Visualization (faster)
```bash
python main.py --scenario apartment --no_viz
```

## Troubleshooting

### If you see import errors:
```bash
# Make sure you're in the right directory
cd "Deadlock Resolution of Connected Multi-Agent Systems using Hierarchical Control"

# Run the test again
python test_installation.py
```

### If visualization doesn't work:
```bash
python main.py --scenario simple --no_viz
```

### If CVXPY/OSQP has issues:
```bash
pip install --upgrade cvxpy osqp
```

## What's Next?

- Read [USAGE.md](USAGE.md) for detailed command-line options
- Read [IMPLEMENTATION_NOTES.md](IMPLEMENTATION_NOTES.md) for technical details
- Explore the code in `src/` directory
- Try creating custom scenarios in `scenarios/scenarios.py`

## Quick Comparison Study

To see the difference the hierarchical control makes:

**Run 1: With hierarchical control**
```bash
python main.py --scenario apartment > results_hierarchical.txt
```

**Run 2: Without hierarchical control (baseline)**
```bash
python main.py --scenario apartment --no_planner > results_baseline.txt
```

Then compare the success rates!

---

**Ready to start?**
```bash
python main.py --scenario simple
```

Enjoy exploring multi-agent hierarchical control! 🤖
