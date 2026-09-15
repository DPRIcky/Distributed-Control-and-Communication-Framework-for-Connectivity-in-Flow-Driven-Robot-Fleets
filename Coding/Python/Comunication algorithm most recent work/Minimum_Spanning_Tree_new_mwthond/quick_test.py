"""Quick test to verify all sizes work."""

import sys
from pathlib import Path
parent_dir = Path(__file__).parent.parent
sys.path.insert(0, str(parent_dir))

import numpy as np
from GHS_simulation_GUI import GHSSimulation
from config import SimulationConfig, ControlConfig


def quick_test():
    """Quick test of convergence."""
    test_cases = [4, 5, 6, 8]
    results = []
    
    for num_robots in test_cases:
        sim_config = SimulationConfig(
            num_robots=num_robots,
            communication_radius=15.0,
            verbose=False
        )
        control_config = ControlConfig()
        
        sim = GHSSimulation(sim_config, control_config)
        
        expected_mst = num_robots - 1
        
        # Run until convergence or max steps
        for step in range(200):
            sim.step()
            if sim.ghs_converged:
                break
        
        final_edges = len(sim.current_edges())
        mst_edges = len(sim.get_mst_edges())
        success = sim.ghs_converged and final_edges == expected_mst and mst_edges == expected_mst
        
        results.append({
            'robots': num_robots,
            'converged': sim.ghs_converged,
            'steps': step + 1,
            'final_edges': final_edges,
            'mst_edges': mst_edges,
            'expected': expected_mst,
            'success': success
        })
        
        status = "✓" if success else "✗"
        print(f"{status} {num_robots} robots: converged={sim.ghs_converged}, edges={final_edges}/{expected_mst}, steps={step+1}")
    
    print("\nSUMMARY:")
    all_success = all(r['success'] for r in results)
    if all_success:
        print("✓✓✓ ALL TESTS PASSED!")
    else:
        print(f"✗✗✗ {sum(1 for r in results if not r['success'])} tests failed")


if __name__ == "__main__":
    quick_test()
