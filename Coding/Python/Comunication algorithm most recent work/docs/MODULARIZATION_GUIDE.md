# Modularized Underwater Robotics Codebase

## ✨ What Changed?

The code has been **fully modularized** from a single 1000-line file into a clean, maintainable package structure. Everything works exactly the same, but the code is now:

- **Easier to understand** - Each module has a single responsibility
- **Easier to test** - Unit tests for each component
- **Easier to extend** - Add new features without breaking existing code
- **Easier to reuse** - Import only what you need

---

## 📁 New Project Structure

```
├── main.py                       # Main entry point (use this!)
├── requirements.txt              # Dependencies
├── setup.py                      # Package installation
├── README.md                     # Original documentation
├── MODULARIZATION_GUIDE.md      # This file
│
├── config/                       # Configuration management
│   ├── simulation_config.py     # Simulation parameters
│   ├── control_config.py        # CLF/CBF control gains
│   ├── consensus_config.py      # Hybrid consensus parameters
│   └── visualization_config.py  # GUI/plotting settings
│
├── core/                         # Core data structures
│   ├── robot.py                 # ChainRobot class
│   ├── flow_field.py            # FlowField model
│   └── types.py                 # Type aliases
│
├── controllers/                  # Control algorithms
│   ├── clf_controller.py        # Control Lyapunov Function
│   ├── cbf_controller.py        # Control Barrier Function
│   └── hybrid_controller.py     # Combined CLF-CBF
│
├── graph/                        # Graph theory utilities
│   ├── topology.py              # Graph connectivity management
│   ├── connectivity_tree.py     # BFS tree building
│   └── edge_analysis.py         # Path finding, redundancy checks
│
├── consensus/                    # Consensus algorithms
│   └── hybrid_pruning.py        # Multi-layer robustness consensus
│
├── simulation/                   # Simulation engine
│   └── simulation_engine.py     # HybridUnderwaterSimulation class
│
├── visualization/                # GUI components
│   └── animator.py              # HybridAnimator class
│
├── tests/                        # Unit tests
│   ├── test_robot.py
│   ├── test_controllers.py
│   ├── test_graph.py
│   └── test_simulation.py
│
└── examples/                     # Example scripts
    ├── basic_simulation.py      # Simple text-mode example
    └── gui_simulation.py        # GUI example
```

---

## 🚀 Quick Start

### Running Simulations

**The old way still works** (with deprecation warning):
```bash
python underwater_consensus_chain_hybrid.py gui
```

**The new, recommended way:**
```bash
# Text mode
python main.py text --robots 8 --steps 100

# GUI mode  
python main.py gui --robots 10

# Quiet mode (suppress verbose output)
python main.py text --quiet
```

### Using as a Library

```python
from config import SimulationConfig, ControlConfig, ConsensusConfig
from simulation import HybridUnderwaterSimulation
from visualization import HybridAnimator

# Configure simulation
sim_config = SimulationConfig(
    num_robots=10,
    communication_radius=3.0,
    verbose=True,
    seed=42  # For reproducibility
)
control_config = ControlConfig(
    clf_gain=0.8,
    cbf_safety_gain=4.0
)
consensus_config = ConsensusConfig(
    stability_threshold=5,
    rolling_window_size=10
)

# Create and run simulation
sim = HybridUnderwaterSimulation(sim_config, control_config, consensus_config)

# Text mode
for step_idx, report in enumerate(sim.iterate(100)):
    if report['decision'] == 'prune':
        print(f"Pruned edge: {report['removed_edge']}")

# GUI mode
import matplotlib.pyplot as plt
animator = HybridAnimator(sim)
anim = animator.animate()
plt.show()
```

---

## 🧪 Running Tests

```bash
# Install pytest if needed
pip install pytest

# Run all tests
pytest tests/

# Run specific test file
pytest tests/test_robot.py -v

# Run with coverage
pip install pytest-cov
pytest tests/ --cov=. --cov-report=html
```

---

## 📦 Installing as a Package

```bash
# Install in development mode (editable)
pip install -e .

# Now you can import from anywhere
python -c "from simulation import HybridUnderwaterSimulation; print('Works!')"
```

---

## 🔧 Customizing the Simulation

### Changing Control Parameters

```python
from config import ControlConfig

# More aggressive goal-seeking
control = ControlConfig(
    clf_gain=1.5,           # Stronger attraction to goal
    cbf_safety_gain=6.0,    # Stronger collision avoidance
    safety_distance=1.5     # Larger safety buffer
)
```

### Changing Consensus Parameters

```python
from config import ConsensusConfig

# More conservative edge pruning
consensus = ConsensusConfig(
    stability_threshold=10,          # Wait longer for stability
    rolling_window_size=20,          # Larger history window
    consensus_rounds_per_step=5,     # More rounds per step
    total_consensus_rounds_needed=100 # Require more agreement
)
```

### Custom Visualization

```python
from config import VisualizationConfig

vis = VisualizationConfig(
    interval_ms=50,         # Faster animation
    figsize=(14, 7),        # Larger window
    flow_alpha=0.6,         # More visible flow field
    active_linewidth=3.5    # Thicker edge lines
)
```

---

## 🎯 Module Responsibilities

### config/
**Purpose**: Centralized parameter management with validation

- `SimulationConfig`: Workspace, robots, communication radius, seed
- `ControlConfig`: CLF/CBF gains, safety distances
- `ConsensusConfig`: Stability thresholds, consensus rounds
- `VisualizationConfig`: Plot appearance, animation speed

### core/
**Purpose**: Fundamental robot and environment models

- `ChainRobot`: Robot state and dynamics
- `FlowField`: Underwater current model
- `types.py`: Shared type definitions (Edge, etc.)

### controllers/
**Purpose**: Separation of control strategies

- `CLFController`: Goal-seeking control
- `CBFController`: Safety and connectivity control
- `HybridCLFCBFController`: Combined controller

**Why separate?** Easy to swap in different control laws, test individually, or use in other projects.

### graph/
**Purpose**: Network topology analysis

- `TopologyManager`: Graph construction from robot positions
- `ConnectivityTree`: BFS-based tree building
- `EdgeAnalyzer`: Redundancy detection, connectivity checks

**Why separate?** Graph algorithms are reusable in any multi-agent system.

### consensus/
**Purpose**: Distributed decision-making

- `HybridPruningManager`: Orchestrates multi-layer robustness consensus

**Why separate?** Consensus algorithm is independent of robot dynamics.

### simulation/
**Purpose**: Main simulation orchestration

- `HybridUnderwaterSimulation`: Coordinates all subsystems

**Why separate?** Clean separation between simulation logic and subsystems.

### visualization/
**Purpose**: GUI rendering

- `HybridAnimator`: Interactive matplotlib animation

**Why separate?** Visualization is optional; simulation can run headless.

---

## 🔄 Migration from Old Code

If you have custom code based on the old monolithic script:

### Before (Old Code)
```python
from underwater_consensus_chain_hybrid import HybridUnderwaterSimulation

sim = HybridUnderwaterSimulation(
    num_robots=10,
    communication_radius=3.0,
    clf_gain=0.8,
    # ... lots of parameters
)
```

### After (New Modular Code)
```python
from config import SimulationConfig, ControlConfig, ConsensusConfig
from simulation import HybridUnderwaterSimulation

sim_config = SimulationConfig(num_robots=10, communication_radius=3.0)
control_config = ControlConfig(clf_gain=0.8)
consensus_config = ConsensusConfig()

sim = HybridUnderwaterSimulation(sim_config, control_config, consensus_config)
```

**Benefits of new approach:**
- Clearer organization of parameters
- Default values in config classes
- Validation built-in
- Easier to see what can be configured

---

## 📊 Benefits Summary

| Aspect | Before | After |
|--------|--------|-------|
| **File Count** | 1 monolithic file (1000 lines) | 20+ focused modules (<200 lines each) |
| **Testability** | Hard to test individual parts | Unit tests for each module |
| **Reusability** | Must copy entire file | Import only what you need |
| **Maintainability** | Find bugs in 1000-line file | Clear module boundaries |
| **Extensibility** | Edit monolithic code | Add new modules without breaking old |
| **Documentation** | One big docstring | Module-specific documentation |
| **Collaboration** | Merge conflicts | Multiple developers, separate modules |

---

## 🚧 Future Enhancements (Easy Now!)

With the new structure, adding features is simple:

### Add a New Controller
```python
# controllers/pid_controller.py
class PIDController:
    def compute_control(self, robot_id, robots):
        # Your PID logic here
        pass

# Then just import and use it
from controllers.pid_controller import PIDController
```

### Add a New Flow Field
```python
# core/vortex_flow.py
class VortexFlow:
    def velocity(self, position, time):
        # Vortex flow model
        pass

# Use in simulation
from core.vortex_flow import VortexFlow
sim.flow = VortexFlow()
```

### Add Performance Logging
```python
# utils/logging.py
class PerformanceLogger:
    def log_step(self, sim_state):
        # Save metrics to file
        pass
```

---

## ❓ FAQ

**Q: Will the old script still work?**  
A: Yes! The old `underwater_consensus_chain_hybrid.py` still works but shows a deprecation warning.

**Q: Is the behavior exactly the same?**  
A: Yes, with the same random seed you'll get identical results.

**Q: Do I need to install anything new?**  
A: No, same dependencies (numpy, matplotlib).

**Q: Can I mix old and new code?**  
A: Yes, but we recommend migrating fully for consistency.

**Q: Where's the documentation?**  
A: Each module has docstrings. Use `help(module_name)` or read the source.

---

## 📝 Contributing

Now that code is modular:

1. **Find the right module** for your change
2. **Write tests** in `tests/`
3. **Update documentation** in the module
4. **Submit changes** knowing they won't break unrelated code

---

## 🙏 Acknowledgments

Modularization completed on 2026-01-05 following the design outlined in `MODULARIZATION_PROPOSAL.md`.

Original research and implementation by the ASU robotics research team.

---

**Questions?** The modular structure follows standard Python package conventions. Each module is self-contained and well-documented.

**Enjoy the cleaner codebase! 🎉**
