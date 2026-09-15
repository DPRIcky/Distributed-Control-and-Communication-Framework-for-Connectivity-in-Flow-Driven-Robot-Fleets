#!/usr/bin/env python3
"""
Quick guide to run the GUI simulation with CBF constraint visualization.

The GUI now shows:
- Critical edge constraint status (ACTIVE / Waiting)
- Max critical edge distance vs constraint margin
- Number of CBF violations
- Live updates of the distributed algorithm
"""

import subprocess
import sys

def run_gui(comm_radius: float = 2.0):
    """Run GUI simulation with specified communication radius."""
    print(f"\n{'='*70}")
    print(f"Starting GUI with communication radius: {comm_radius}m")
    print(f"{'='*70}\n")
    
    # Run the GUI simulation
    script_path = "Critical_edge_detection_baseline/gui_simulation.py"
    try:
        subprocess.run(
            [sys.executable, script_path, str(comm_radius)],
            cwd=".",
            check=False
        )
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    if len(sys.argv) > 1:
        try:
            radius = float(sys.argv[1])
            run_gui(radius)
        except ValueError:
            print("Invalid radius value")
    else:
        # Show menu
        print("\n" + "="*70)
        print("CBF Constraint Visualization - GUI Simulator")
        print("="*70)
        print("\nUsage:")
        print("  python run_gui.py [comm_radius]")
        print("\nPre-configured options:")
        print("  python run_gui.py 1.0   # Test CBF with tight constraints (1m radius)")
        print("  python run_gui.py 2.0   # Optimal performance (2m radius)")
        print("  python run_gui.py 3.0   # Stable operation (3m radius)")
        print("\nOr directly:")
        print("  cd Critical_edge_detection_baseline")
        print("  python gui_simulation.py 1.0")
        print("\n" + "="*70)
