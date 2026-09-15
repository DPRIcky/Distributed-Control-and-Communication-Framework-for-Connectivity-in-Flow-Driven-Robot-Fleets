#!/usr/bin/env python3
"""Diagnose connectivity issues."""

import sys
import os
import io
import contextlib
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent))

# Suppress warnings
with contextlib.redirect_stdout(io.StringIO()):
    from config import SimulationConfig, ControlConfig
    from Critical_edge_detection_baseline.simple_simulation import CriticalEdgeSimulation
import numpy as np

def diagnose():
    """Diagnose what's happening at the end of simulation."""
    
    print("=" * 70)
    print("Connectivity Diagnosis")
    print("=" * 70)
    
    sim_config = SimulationConfig(
        num_robots=8,
        communication_radius=1.0,
        workspace_size=[10.0, 10.0],
        dt=0.01,
        verbose=False,
        seed=42
    )
    
    control_config = ControlConfig(
        cbf_connectivity_gain=15.0,  # Increased gain
        max_control_force=8.0  # Increased force limit
    )
    
    print(f"Configuration:")
    print(f"  Robots: 8")
    print(f"  Comm radius: 1.0m")
    print(f"  CBF gain: {control_config.cbf_connectivity_gain}")
    print(f"  Max force: {control_config.max_control_force}")
    
    sim = CriticalEdgeSimulation(
        sim_config=sim_config,
        control_config=control_config,
        update_frequency=10,
        prefer_critical=True,
        verbose=False
    )
    
    # Run 100 steps
    print(f"\nRunning 100 steps...")
    for step in range(100):
        sim.step()
        if (step + 1) % 25 == 0:
            print(f"  Step {step + 1}: Running...")
    
    # Now diagnose
    print("\n" + "=" * 70)
    print("END STATE DIAGNOSIS")
    print("=" * 70)
    
    edge_lengths = sim._get_edge_lengths()
    print(f"\nAvailable edges in topology: {len(edge_lengths)}")
    print(f"MST size: {len(sim.baseline.current_mst)}")
    print(f"Critical edges identified: {len(sim.baseline.critical_edges)}")
    print(f"Total edges pruned: {sim.edges_pruned_count}")
    
    print(f"\nRobot positions:")
    for i, robot in enumerate(sim.robots):
        print(f"  Robot {i}: {robot.position}")
    
    print(f"\nDistances between all robot pairs (comm_radius={sim_config.communication_radius}m):")
    constraint_margin = sim_config.communication_radius * 0.95
    for i in range(sim.num_robots):
        for j in range(i+1, sim.num_robots):
            dist = np.linalg.norm(sim.robots[i].position - sim.robots[j].position)
            status = "✓" if dist <= sim_config.communication_radius else "✗"
            in_critical = (i, j) in sim.baseline.critical_edges or (j, i) in sim.baseline.critical_edges
            print(f"  ({i},{j}): {dist:.4f}m {status} {'[CRITICAL]' if in_critical else ''}")
    
    # Check how many pairs are within comm radius
    connected_pairs = 0
    critical_pairs = 0
    for i in range(sim.num_robots):
        for j in range(i+1, sim.num_robots):
            dist = np.linalg.norm(sim.robots[i].position - sim.robots[j].position)
            if dist <= sim_config.communication_radius:
                connected_pairs += 1
            if (i, j) in sim.baseline.critical_edges or (j, i) in sim.baseline.critical_edges:
                critical_pairs += 1
    
    print(f"\nSummary:")
    print(f"  Robot pairs within comm_radius: {connected_pairs}/{sim.num_robots * (sim.num_robots - 1) // 2}")
    print(f"  Critical edge pairs: {critical_pairs}")
    print(f"  MST needs: {sim.num_robots - 1} edges for connectivity")
    
    # Determine if network is actually connected
    if connected_pairs >= sim.num_robots - 1:
        print(f"\n✓ Network topology allows MST - sufficient edges exist within comm_radius")
    else:
        print(f"\n✗ Network fragmented - only {connected_pairs} edges possible (need {sim.num_robots - 1})")

if __name__ == "__main__":
    try:
        diagnose()
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
