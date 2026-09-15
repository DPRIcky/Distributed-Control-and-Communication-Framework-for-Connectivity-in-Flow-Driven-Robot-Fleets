"""Comprehensive stability test for GHS."""
import subprocess
import re

test_cases = [
    (4, "small"),
    (5, "small"),
    (6, "medium"),
    (8, "medium"),
    (10, "large"),
    (15, "large"),
    (20, "xlarge")
]

print("="*70)
print("GHS COMPREHENSIVE STABILITY TEST")
print("="*70)

results = []

for num_robots, size in test_cases:
    print(f"\nTesting {num_robots} robots ({size})...", end=" ", flush=True)
    
    # Create a simple test script
    test_code = f"""
import sys
from pathlib import Path
parent_dir = Path(__file__).parent.parent
sys.path.insert(0, str(parent_dir))

from GHS_simulation_GUI import GHSSimulation
from config import SimulationConfig, ControlConfig
import numpy as np

np.random.seed(42)
sim_config = SimulationConfig(num_robots={num_robots}, communication_radius=2.0, verbose=False)
control_config = ControlConfig()
sim = GHSSimulation(sim_config, control_config)

for _ in range(500):
    sim.step()
    if sim.ghs_converged:
        print(f"CONVERGED: {{sim._count_mst_edges()}}/{{sim.num_robots-1}} edges")
        sys.exit(0)

print(f"FAILED: {{sim._count_mst_edges()}}/{{sim.num_robots-1}} edges, {{len(set(sim.fragment_id))}} fragments")
sys.exit(1)
"""
    
    try:
        result = subprocess.run(
            ["python", "-c", test_code],
            capture_output=True,
            text=True,
            timeout=60,
            cwd=r"c:\Users\prajj\OneDrive - Arizona State University\ASU\PhD\Research\Coding\Python\Comunication algorithm most recent work\Minimum_Spanning_Tree_new_mwthond"
        )
        
        if "CONVERGED" in result.stdout:
            match = re.search(r'CONVERGED: (\d+)/(\d+)', result.stdout)
            if match:
                edges = match.group(1)
                target = match.group(2)
                print(f"✓ PASS ({edges}/{target} edges)")
                results.append((num_robots, True, f"{edges}/{target}"))
        else:
            match = re.search(r'FAILED: (\d+)/(\d+) edges, (\d+) fragments', result.stdout)
            if match:
                edges = match.group(1)
                target = match.group(2)
                frags = match.group(3)
                print(f"✗ FAIL ({edges}/{target} edges, {frags} frags)")
                results.append((num_robots, False, f"{edges}/{target}, {frags} frags"))
            else:
                print(f"✗ ERROR")
                results.append((num_robots, False, "error"))
    except subprocess.TimeoutExpired:
        print(f"✗ TIMEOUT")
        results.append((num_robots, False, "timeout"))
    except Exception as e:
        print(f"✗ ERROR: {e}")
        results.append((num_robots, False, str(e)[:20]))

print("\n" + "="*70)
print("SUMMARY")
print("="*70)

passed = sum(1 for _, success, _ in results if success)
total = len(results)

for num_robots, success, detail in results:
    status = "✓ PASS" if success else "✗ FAIL"
    print(f"{num_robots:2d} robots: {status:8s} - {detail}")

print(f"\nOverall: {passed}/{total} tests passed ({100*passed//total}%)")
