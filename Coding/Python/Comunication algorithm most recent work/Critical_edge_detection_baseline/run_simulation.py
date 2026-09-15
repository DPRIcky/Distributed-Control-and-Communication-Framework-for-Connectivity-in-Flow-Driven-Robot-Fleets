"""
Run Critical Edge Detection Baseline Simulation

This script provides a flexible interface for running simulations with the
distributed critical edge detection algorithm.

Usage:
    python run_simulation.py                    # Default parameters
    python run_simulation.py --robots 15        # Custom robot count
    python run_simulation.py --steps 500        # Custom step count
    python run_simulation.py --save results.npz # Save results
    python run_simulation.py --visualize        # Enable visualization
"""

import sys
import os
import argparse
import numpy as np
from pathlib import Path
import io
import contextlib

# Add parent directory to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config import SimulationConfig, ControlConfig

# Suppress import warnings during initialization (like GUI simulation does)
with contextlib.redirect_stdout(io.StringIO()):
    from Critical_edge_detection_baseline.simple_simulation import CriticalEdgeSimulation


def parse_args():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description='Run Critical Edge Detection Baseline Simulation',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    
    # Simulation parameters
    parser.add_argument('--robots', type=int, default=10,
                        help='Number of robots')
    parser.add_argument('--steps', type=int, default=500,
                        help='Number of simulation steps')
    parser.add_argument('--dt', type=float, default=0.05,
                        help='Time step (seconds)')
    parser.add_argument('--comm-radius', type=float, default=3.0,
                        help='Communication radius')
    
    # Workspace
    parser.add_argument('--workspace', type=float, nargs=2, default=[10.0, 10.0],
                        help='Workspace size (width height)')
    
    # Algorithm parameters
    parser.add_argument('--update-freq', type=int, default=10,
                        help='MST update frequency (steps)')
    parser.add_argument('--no-prefer-critical', action='store_true',
                        help='Do not prioritize critical edges')
    
    # Output options
    parser.add_argument('--save', type=str, default=None,
                        help='Save results to file (.npz)')
    parser.add_argument('--verbose', action='store_true',
                        help='Print detailed output')
    parser.add_argument('--quiet', action='store_true',
                        help='Minimal output')
    
    # Visualization
    parser.add_argument('--visualize', action='store_true',
                        help='Enable real-time visualization (requires matplotlib)')
    parser.add_argument('--plot-interval', type=int, default=50,
                        help='Visualization update interval')
    
    # Other
    parser.add_argument('--seed', type=int, default=None,
                        help='Random seed for reproducibility')
    
    return parser.parse_args()


def setup_simulation(args):
    """Create and configure simulation."""
    # Create configuration
    sim_config = SimulationConfig(
        num_robots=args.robots,
        communication_radius=args.comm_radius,
        dt=args.dt,
        workspace_size=tuple(args.workspace),
        verbose=args.verbose and not args.quiet,
        seed=args.seed
    )
    
    control_config = ControlConfig()
    
    # Create simulation
    sim = CriticalEdgeSimulation(
        sim_config=sim_config,
        control_config=control_config,
        update_frequency=args.update_freq,
        prefer_critical=not args.no_prefer_critical,
        verbose=args.verbose and not args.quiet
    )
    
    return sim


def setup_visualization(sim, args):
    """Setup matplotlib visualization if requested."""
    if not args.visualize:
        return None
    
    try:
        import matplotlib.pyplot as plt
        from matplotlib.patches import Circle
        
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
        
        # Setup plot 1: Robot positions and edges
        ax1.set_xlim(0, args.workspace[0])
        ax1.set_ylim(0, args.workspace[1])
        ax1.set_aspect('equal')
        ax1.set_title('Critical Edge Detection - Robot Network')
        ax1.set_xlabel('X Position (m)')
        ax1.set_ylabel('Y Position (m)')
        ax1.grid(True, alpha=0.3)
        
        # Setup plot 2: Metrics over time
        ax2.set_title('Simulation Metrics')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Count')
        ax2.grid(True, alpha=0.3)
        
        plt.ion()
        plt.show()
        
        return {'fig': fig, 'ax1': ax1, 'ax2': ax2, 
                'time_data': [], 'edge_data': [], 'pruned_data': []}
    
    except ImportError:
        print("Warning: matplotlib not available, visualization disabled")
        return None


def update_visualization(sim, vis, step):
    """Update visualization plots."""
    if vis is None:
        return
    
    try:
        import matplotlib.pyplot as plt
        from matplotlib.patches import Circle
        
        ax1, ax2 = vis['ax1'], vis['ax2']
        
        # Clear previous frame
        ax1.clear()
        ax1.set_xlim(0, sim.workspace[0])
        ax1.set_ylim(0, sim.workspace[1])
        ax1.set_aspect('equal')
        ax1.set_title(f'Critical Edge Detection - t={sim.time:.2f}s')
        ax1.set_xlabel('X Position (m)')
        ax1.set_ylabel('Y Position (m)')
        ax1.grid(True, alpha=0.3)
        
        # Draw communication ranges
        for robot in sim.robots:
            circle = Circle(robot.position, sim.communication_radius, 
                          fill=False, color='lightblue', alpha=0.2, linestyle='--')
            ax1.add_patch(circle)
        
        # Draw edges
        edge_lengths = sim._get_edge_lengths()
        mst_edges = sim.baseline.current_mst
        
        for (i, j), length in edge_lengths.items():
            pos_i = sim.robots[i].position
            pos_j = sim.robots[j].position
            
            # Check if edge is in MST
            is_mst = ((i, j) in mst_edges or (j, i) in mst_edges)
            is_critical = ((i, j) in sim.baseline.critical_edges or 
                         (j, i) in sim.baseline.critical_edges)
            
            if is_critical:
                color, width, alpha = 'red', 2.5, 0.9  # Critical edges
            elif is_mst:
                color, width, alpha = 'green', 2.0, 0.7  # MST edges
            else:
                color, width, alpha = 'gray', 1.0, 0.3  # Other edges
            
            ax1.plot([pos_i[0], pos_j[0]], [pos_i[1], pos_j[1]], 
                    color=color, linewidth=width, alpha=alpha)
        
        # Draw robots
        positions = np.array([robot.position for robot in sim.robots])
        ax1.scatter(positions[:, 0], positions[:, 1], 
                   c='blue', s=100, zorder=10, edgecolors='black', linewidths=1.5)
        
        # Draw goal
        ax1.scatter(sim.goal_position[0], sim.goal_position[1], 
                   c='gold', s=200, marker='*', zorder=10, 
                   edgecolors='black', linewidths=1.5, label='Goal')
        
        # Add legend
        from matplotlib.lines import Line2D
        legend_elements = [
            Line2D([0], [0], color='red', linewidth=2.5, label='Critical edges'),
            Line2D([0], [0], color='green', linewidth=2.0, label='MST edges'),
            Line2D([0], [0], color='gray', linewidth=1.0, label='Other edges'),
        ]
        ax1.legend(handles=legend_elements, loc='upper right')
        
        # Update metrics plot
        vis['time_data'].append(sim.time)
        vis['edge_data'].append(sim.total_edges)
        vis['pruned_data'].append(sim.edges_pruned_count)
        
        ax2.clear()
        ax2.plot(vis['time_data'], vis['edge_data'], 'b-', label='Current edges', linewidth=2)
        ax2.axhline(y=sim.num_robots - 1, color='g', linestyle='--', 
                   label=f'MST target ({sim.num_robots - 1})', linewidth=2)
        ax2.plot(vis['time_data'], vis['pruned_data'], 'r-', 
                label='Total pruned', linewidth=2, alpha=0.7)
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Count')
        ax2.set_title('Simulation Metrics')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        plt.pause(0.01)
    
    except Exception as e:
        print(f"Visualization error: {e}")


def run_simulation(args):
    """Run the simulation with given parameters."""
    # Print header
    if not args.quiet:
        print("=" * 80)
        print("CRITICAL EDGE DETECTION BASELINE SIMULATION")
        print("=" * 80)
        print(f"Configuration:")
        print(f"  Robots: {args.robots}")
        print(f"  Steps: {args.steps}")
        print(f"  Time step: {args.dt}s")
        print(f"  Communication radius: {args.comm_radius}m")
        print(f"  Workspace: {args.workspace[0]}m × {args.workspace[1]}m")
        print(f"  MST update frequency: every {args.update_freq} steps")
        print(f"  Prefer critical edges: {not args.no_prefer_critical}")
        if args.seed is not None:
            print(f"  Random seed: {args.seed}")
        print("=" * 80)
    
    # Setup simulation
    sim = setup_simulation(args)
    
    # Setup visualization
    vis = setup_visualization(sim, args)
    
    # Storage for results
    results = {
        'time': [],
        'edges': [],
        'positions': [],
        'mst_updates': [],
        'edges_pruned': []
    }
    
    # Run simulation
    if not args.quiet:
        print("\nRunning simulation...")
    
    try:
        for step in range(args.steps):
            # Step simulation
            sim.step()
            
            # Record data
            results['time'].append(sim.time)
            results['edges'].append(sim.total_edges)
            results['positions'].append(np.array([r.position for r in sim.robots]))
            
            stats = sim.baseline.get_statistics()
            results['mst_updates'].append(stats['mst_updates'])
            results['edges_pruned'].append(sim.edges_pruned_count)
            
            # Update visualization
            if vis and step % args.plot_interval == 0:
                update_visualization(sim, vis, step)
            
            # Print progress
            if not args.quiet and step % 100 == 0 and step > 0:
                print(f"  Step {step:4d} | t={sim.time:6.2f}s | "
                      f"Edges: {sim.total_edges:3d} | "
                      f"Pruned: {sim.edges_pruned_count:3d} | "
                      f"MST updates: {stats['mst_updates']:3d}")
    
    except KeyboardInterrupt:
        print("\n\nSimulation interrupted by user")
    
    # Final visualization update
    if vis:
        update_visualization(sim, vis, args.steps)
        print("\nClose the plot window to continue...")
        try:
            import matplotlib.pyplot as plt
            plt.ioff()
            plt.show()
        except:
            pass
    
    # Print final statistics
    if not args.quiet:
        print("\n" + "=" * 80)
        print("SIMULATION COMPLETE")
        print("=" * 80)
        sim.print_statistics()
    
    # Save results if requested
    if args.save:
        save_path = Path(args.save)
        save_path.parent.mkdir(parents=True, exist_ok=True)
        
        np.savez(
            save_path,
            time=np.array(results['time']),
            edges=np.array(results['edges']),
            positions=np.array(results['positions']),
            mst_updates=np.array(results['mst_updates']),
            edges_pruned=np.array(results['edges_pruned']),
            config={
                'num_robots': args.robots,
                'steps': args.steps,
                'dt': args.dt,
                'comm_radius': args.comm_radius,
                'workspace': args.workspace,
                'update_freq': args.update_freq,
                'seed': args.seed
            }
        )
        
        if not args.quiet:
            print(f"\nResults saved to: {save_path}")
    
    return sim, results


def main():
    """Main entry point."""
    args = parse_args()
    
    try:
        sim, results = run_simulation(args)
        return 0
    except Exception as e:
        print(f"\nError: {e}", file=sys.stderr)
        if args.verbose:
            import traceback
            traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())
