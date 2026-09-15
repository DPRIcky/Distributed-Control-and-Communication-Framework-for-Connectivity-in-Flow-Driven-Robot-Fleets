"""Test GHS with 15 robots to verify deadlock recovery."""
import sys
from pathlib import Path
parent_dir = Path(__file__).parent.parent
sys.path.insert(0, str(parent_dir))

from GHS_simulation_GUI import GHSSimulation
from config import SimulationConfig, ControlConfig
import numpy as np

def test_15_robots():
    """Test GHS convergence with 15 robots."""
    print("\n" + "="*70)
    print("Testing GHS with 15 robots (deadlock recovery test)")
    print("="*70 + "\n")
    
    # Use same random seed as the GUI test for reproducibility
    np.random.seed(42)
    
    sim_config = SimulationConfig(
        num_robots=15,
        communication_radius=2.0,
        verbose=False
    )
    control_config = ControlConfig()
    
    sim = GHSSimulation(sim_config, control_config)
    
    # Run simulation for up to 100 seconds or 2000 steps
    max_steps = 2000
    max_time = 100.0
    
    for step in range(max_steps):
        sim.step()
        
        if sim.ghs_converged:
            print(f"\n✓ 15 robots: converged=True, edges={sim._count_mst_edges()}/{sim.num_robots-1}, steps={step+1}")
            print(f"   Convergence time: {sim.convergence_time:.2f}s")
            print(f"   Messages processed: {sim.messages_processed}")
            print(f"   Edges pruned: {sim.pruned_edge_count}/{sim.initial_edge_count}")
            return True
        
        if sim.time > max_time:
            break
    
    print(f"\n✗ 15 robots: converged=False after {max_steps} steps")
    print(f"   Final state: {sim._count_mst_edges()}/{sim.num_robots-1} MST edges")
    print(f"   Fragments: {len(set(sim.fragment_id))}")
    print(f"   Messages processed: {sim.messages_processed}")
    return False

if __name__ == "__main__":
    success = test_15_robots()
    sys.exit(0 if success else 1)
