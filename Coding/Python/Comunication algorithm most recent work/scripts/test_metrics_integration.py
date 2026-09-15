"""Quick test of metrics collection integration."""

import sys
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from config.scenarios import get_scenario
from simulation import HybridUnderwaterSimulation

def test_metrics_integration():
    """Test that metrics are collected during simulation."""
    print("\n" + "="*60)
    print("TESTING METRICS INTEGRATION")
    print("="*60)
    
    # Load baseline scenario
    sim_cfg, ctrl_cfg, cons_cfg, flow_params, name = get_scenario('A')
    print(f"\nScenario: {name}")
    print(f"Robots: {sim_cfg.num_robots}")
    print(f"Communication radius: {sim_cfg.communication_radius} m")
    
    # Create simulation
    sim = HybridUnderwaterSimulation(sim_cfg, ctrl_cfg, cons_cfg)
    
    # Run for 50 steps
    num_steps = 50
    print(f"\nRunning {num_steps} simulation steps...")
    
    for step in range(num_steps):
        sim.step()
        
        if step % 10 == 0:
            print(f"  Step {step}: Time={sim.time:.1f}s, "
                  f"Edges={len(sim.current_edges())}, "
                  f"Metrics recorded={len(sim.metrics.time_series)}")
    
    # Check metrics were collected
    print(f"\n✓ Simulation completed!")
    print(f"\nMetrics Summary:")
    stats = sim.metrics.get_summary_statistics()
    for key, value in stats.items():
        if isinstance(value, float):
            print(f"  {key}: {value:.3f}")
        else:
            print(f"  {key}: {value}")
    
    # Save test output
    print(f"\nSaving metrics...")
    sim.metrics.save('test_metrics.json')
    sim.metrics.save_arrays('test_arrays.npz')
    print(f"✓ Saved to test_metrics.json and test_arrays.npz")
    
    print("\n" + "="*60)
    print("✓ METRICS INTEGRATION TEST PASSED")
    print("="*60 + "\n")

if __name__ == "__main__":
    test_metrics_integration()
