"""
Example custom scenarios for underwater consensus with CLF-CBF control.

This file demonstrates how to configure different simulation scenarios
with various goal positions, control parameters, and robot counts.
"""

import numpy as np
from underwater_consensus_chain import UnderwaterChainSimulation, ChainAnimator
import matplotlib.pyplot as plt


def scenario_1_close_goal():
    """Scenario 1: Close goal with strong CLF - Robots should reach goal quickly."""
    print("=" * 70)
    print("SCENARIO 1: Close Goal with Strong CLF")
    print("=" * 70)
    
    sim = UnderwaterChainSimulation(
        num_robots=8,
        communication_radius=2.5,
        goal_position=np.array([4.0, 5.0]),  # Close to starting position
        clf_gain=3.0,  # Strong goal attraction
        cbf_safety_gain=3.0,
        cbf_connectivity_gain=2.0,
        safety_distance=0.3,
        verbose=False
    )
    
    print(f"Goal position: {sim.goal_position}")
    print(f"CLF gain: {sim.clf_gain} (strong)")
    print(f"Starting position: ~[2.5, 5.0]")
    print(f"Expected: Robots should reach goal within 10-15 steps")
    print()
    
    # Run text simulation
    for step in range(30):
        sim.step()
        if step % 5 == 0:
            avg_dist = np.mean([
                np.linalg.norm(r.position - sim.goal_position) 
                for r in sim.robots
            ])
            at_goal = sum(
                1 for r in sim.robots 
                if np.linalg.norm(r.position - sim.goal_position) < 0.5
            )
            print(f"Step {step:2d}: avg_dist={avg_dist:.2f}, at_goal={at_goal}/{sim.num_robots}, edges={len(sim.current_edges())}")
    
    print("\n✓ Scenario 1 complete\n")


def scenario_2_distant_goal():
    """Scenario 2: Distant goal with moderate CLF - Longer journey."""
    print("=" * 70)
    print("SCENARIO 2: Distant Goal with Moderate CLF")
    print("=" * 70)
    
    sim = UnderwaterChainSimulation(
        num_robots=10,
        communication_radius=2.2,
        goal_position=np.array([9.0, 9.0]),  # Far corner
        clf_gain=1.5,  # Moderate goal attraction
        cbf_safety_gain=3.0,
        cbf_connectivity_gain=2.5,  # Stronger connectivity
        safety_distance=0.3,
        verbose=False
    )
    
    print(f"Goal position: {sim.goal_position}")
    print(f"CLF gain: {sim.clf_gain} (moderate)")
    print(f"Starting position: ~[2.5, 5.0]")
    print(f"Expected: Longer journey with strong connectivity maintenance")
    print()
    
    # Run text simulation
    for step in range(50):
        sim.step()
        if step % 10 == 0:
            avg_dist = np.mean([
                np.linalg.norm(r.position - sim.goal_position) 
                for r in sim.robots
            ])
            at_goal = sum(
                1 for r in sim.robots 
                if np.linalg.norm(r.position - sim.goal_position) < 0.5
            )
            print(f"Step {step:2d}: avg_dist={avg_dist:.2f}, at_goal={at_goal}/{sim.num_robots}, edges={len(sim.current_edges())}")
    
    print("\n✓ Scenario 2 complete\n")


def scenario_3_tight_formation():
    """Scenario 3: Tight formation with strong connectivity."""
    print("=" * 70)
    print("SCENARIO 3: Tight Formation with Strong Connectivity")
    print("=" * 70)
    
    sim = UnderwaterChainSimulation(
        num_robots=12,
        communication_radius=1.8,  # Smaller comm range
        goal_position=np.array([6.0, 6.0]),
        clf_gain=1.0,  # Weak goal attraction
        cbf_safety_gain=4.0,  # Strong safety
        cbf_connectivity_gain=4.0,  # Very strong connectivity
        safety_distance=0.25,  # Smaller safety distance
        verbose=False
    )
    
    print(f"Goal position: {sim.goal_position}")
    print(f"Communication radius: {sim.communication_radius} (small)")
    print(f"CBF connectivity gain: {sim.cbf_connectivity_gain} (very strong)")
    print(f"Expected: Robots stay very close together")
    print()
    
    # Run text simulation
    for step in range(40):
        sim.step()
        if step % 10 == 0:
            # Calculate formation spread
            positions = np.array([r.position for r in sim.robots])
            center = np.mean(positions, axis=0)
            spread = np.mean([np.linalg.norm(p - center) for p in positions])
            
            avg_dist = np.mean([
                np.linalg.norm(r.position - sim.goal_position) 
                for r in sim.robots
            ])
            print(f"Step {step:2d}: avg_dist={avg_dist:.2f}, formation_spread={spread:.2f}, edges={len(sim.current_edges())}")
    
    print("\n✓ Scenario 3 complete\n")


def scenario_4_aggressive_goal_seeking():
    """Scenario 4: Aggressive goal-seeking with relaxed connectivity."""
    print("=" * 70)
    print("SCENARIO 4: Aggressive Goal-Seeking (Relaxed Connectivity)")
    print("=" * 70)
    
    sim = UnderwaterChainSimulation(
        num_robots=6,
        communication_radius=3.0,  # Larger comm range
        goal_position=np.array([8.0, 4.0]),
        clf_gain=4.0,  # Very strong goal attraction
        cbf_safety_gain=2.0,
        cbf_connectivity_gain=1.0,  # Weak connectivity
        safety_distance=0.4,
        verbose=False
    )
    
    print(f"Goal position: {sim.goal_position}")
    print(f"CLF gain: {sim.clf_gain} (very strong)")
    print(f"CBF connectivity gain: {sim.cbf_connectivity_gain} (weak)")
    print(f"Expected: Fast goal approach, may stretch connectivity")
    print()
    
    # Run text simulation
    for step in range(30):
        sim.step()
        if step % 5 == 0:
            avg_dist = np.mean([
                np.linalg.norm(r.position - sim.goal_position) 
                for r in sim.robots
            ])
            at_goal = sum(
                1 for r in sim.robots 
                if np.linalg.norm(r.position - sim.goal_position) < 0.5
            )
            
            # Check connectivity
            connected = sum(1 for r in sim.robots if r.parent is not None or r.robot_id == 0)
            
            print(f"Step {step:2d}: avg_dist={avg_dist:.2f}, at_goal={at_goal}/{sim.num_robots}, connected={connected}/{sim.num_robots}")
    
    print("\n✓ Scenario 4 complete\n")


def scenario_5_animated_demo():
    """Scenario 5: Animated demonstration with balanced parameters."""
    print("=" * 70)
    print("SCENARIO 5: Animated Demo (Balanced Parameters)")
    print("=" * 70)
    
    sim = UnderwaterChainSimulation(
        num_robots=8,
        communication_radius=2.5,
        goal_position=np.array([7.5, 5.0]),
        clf_gain=1.5,
        cbf_safety_gain=3.0,
        cbf_connectivity_gain=2.0,
        safety_distance=0.3,
        verbose=True
    )
    
    print(f"Goal position: {sim.goal_position}")
    print(f"Parameters: Balanced for stability and progress")
    print(f"Launching animation...")
    print()
    
    animator = ChainAnimator(sim, interval_ms=150)
    anim = animator.animate()
    plt.tight_layout()
    plt.show()
    
    print("\n✓ Scenario 5 complete\n")


def run_all_text_scenarios():
    """Run all text-based scenarios sequentially."""
    scenario_1_close_goal()
    scenario_2_distant_goal()
    scenario_3_tight_formation()
    scenario_4_aggressive_goal_seeking()


if __name__ == "__main__":
    print("\n" + "=" * 70)
    print("CUSTOM SCENARIO DEMONSTRATIONS")
    print("Underwater Consensus with CLF-CBF Goal-Seeking Control")
    print("=" * 70 + "\n")
    
    # Ask user which scenario to run
    print("Available scenarios:")
    print("  1. Close goal with strong CLF")
    print("  2. Distant goal with moderate CLF")
    print("  3. Tight formation with strong connectivity")
    print("  4. Aggressive goal-seeking (relaxed connectivity)")
    print("  5. Animated demo (balanced parameters)")
    print("  6. Run all text scenarios (1-4)")
    print()
    
    try:
        choice = input("Enter scenario number (1-6) [default: 5]: ").strip()
    except EOFError:
        choice = "5"
    
    if not choice:
        choice = "5"
    
    print()
    
    if choice == "1":
        scenario_1_close_goal()
    elif choice == "2":
        scenario_2_distant_goal()
    elif choice == "3":
        scenario_3_tight_formation()
    elif choice == "4":
        scenario_4_aggressive_goal_seeking()
    elif choice == "5":
        scenario_5_animated_demo()
    elif choice == "6":
        run_all_text_scenarios()
    else:
        print(f"Invalid choice: {choice}")
        print("Running default scenario 5...")
        scenario_5_animated_demo()
    
    print("\n" + "=" * 70)
    print("ALL SCENARIOS COMPLETE")
    print("=" * 70)
