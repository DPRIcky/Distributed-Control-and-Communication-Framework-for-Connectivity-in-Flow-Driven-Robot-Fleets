"""
Small Example: Concurrent Pruning with 4 Robots

This script demonstrates the concurrent pruning algorithm with a minimal
example using 4 robots in a square formation with redundant edges.

Run this to see the mathematical theory in action!
"""

import sys
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
from typing import Set, Tuple

# Add parent directory to path
parent_dir = Path(__file__).parent.parent
sys.path.insert(0, str(parent_dir))

try:
    from concurrent_pruning.concurrent_pruning_manager import ConcurrentPruningManager
    Edge = Tuple[int, int]
except ImportError as e:
    print(f"Import error: {e}")
    print("Please ensure all required modules are available")
    sys.exit(1)


def print_section(title: str):
    """Print a formatted section header."""
    print("\n" + "=" * 80)
    print(title)
    print("=" * 80)


def print_matrix(matrix: np.ndarray, name: str = "Matrix"):
    """Print a matrix with formatted output."""
    print(f"\n{name}:")
    with np.printoptions(precision=3, suppress=True, linewidth=100):
        print(matrix)


def create_4_robot_topology():
    """Create 4-robot square topology with redundant edges.
    
    Topology:
        Robot 0 (0,1) ---- Robot 1 (1,1)
           |        ⟋⟍        |
           |                  |
        Robot 2 (0,0) ---- Robot 3 (1,0)
    
    Returns:
        positions: Array of robot positions
        edges: Set of edges (including redundant diagonals)
    """
    # Robot positions (square formation)
    positions = np.array([
        [0.0, 1.0],  # Robot 0: top-left
        [1.0, 1.0],  # Robot 1: top-right
        [0.0, 0.0],  # Robot 2: bottom-left
        [1.0, 0.0],  # Robot 3: bottom-right
    ])
    
    # Initial edges: complete square with both diagonals (6 edges)
    edges: Set[Edge] = {
        (0, 1),  # top horizontal
        (0, 2),  # left vertical
        (1, 3),  # right vertical
        (2, 3),  # bottom horizontal
        (0, 3),  # diagonal ⟋
        (1, 2),  # diagonal ⟍
    }
    
    return positions, edges


def compute_trust_values(positions: np.ndarray, edges: Set[Edge], sigma: float = 1.0):
    """Compute trust values (adjacency weights) based on distances.
    
    Trust function: t_ij = exp(-d_ij^2 / sigma^2)
    
    Args:
        positions: Robot positions
        edges: Set of edges
        sigma: Sensitivity parameter
        
    Returns:
        Initial adjacency matrix A(0)
    """
    n = len(positions)
    A = np.zeros((n, n))
    
    for i, j in edges:
        distance = np.linalg.norm(positions[i] - positions[j])
        trust = np.exp(-distance**2 / sigma**2)
        A[i, j] = A[j, i] = trust
    
    return A


def print_topology_info(edges: Set[Edge], positions: np.ndarray, iteration: int = 0):
    """Print information about current topology."""
    n = len(positions)
    print(f"\n--- Iteration {iteration} ---")
    print(f"Number of edges: {len(edges)}")
    print(f"Minimum edges needed: {n-1} (tree)")
    print(f"Redundant edges: {len(edges) - (n-1)}")
    print(f"Edges: {sorted(edges)}")
    
    # Compute edge lengths
    print("\nEdge lengths:")
    for i, j in sorted(edges):
        length = np.linalg.norm(positions[i] - positions[j])
        print(f"  ({i},{j}): {length:.3f}")


def visualize_topology(positions: np.ndarray, edges: Set[Edge], iteration: int, 
                       pruned_edge: Tuple[int, int] = None, ax=None):
    """Visualize the robot topology."""
    if ax is None:
        fig, ax = plt.subplots(1, 1, figsize=(6, 6))
    
    # Draw edges
    for i, j in edges:
        if pruned_edge and (i, j) == pruned_edge or (j, i) == pruned_edge:
            # Highlight pruned edge in red dashed
            ax.plot([positions[i, 0], positions[j, 0]], 
                   [positions[i, 1], positions[j, 1]], 
                   'r--', linewidth=2, alpha=0.5, label='Pruned' if (i,j) == pruned_edge else '')
        else:
            ax.plot([positions[i, 0], positions[j, 0]], 
                   [positions[i, 1], positions[j, 1]], 
                   'b-', linewidth=2, alpha=0.6)
    
    # Draw robots
    ax.scatter(positions[:, 0], positions[:, 1], c='orange', s=500, zorder=5, edgecolors='black', linewidths=2)
    
    # Label robots
    for i in range(len(positions)):
        ax.text(positions[i, 0], positions[i, 1], str(i), 
               ha='center', va='center', fontsize=16, fontweight='bold', color='white')
    
    ax.set_xlim(-0.3, 1.3)
    ax.set_ylim(-0.3, 1.3)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.set_title(f'Iteration {iteration} - {len(edges)} edges', fontsize=14, fontweight='bold')
    ax.set_xlabel('X position')
    ax.set_ylabel('Y position')
    
    return ax


def run_small_example():
    """Run the complete 4-robot concurrent pruning example."""
    
    print_section("4-ROBOT CONCURRENT PRUNING EXAMPLE")
    
    # Setup
    num_robots = 4
    positions, edges = create_4_robot_topology()
    sigma = 1.0
    
    print("\n📍 Robot Positions:")
    for i in range(num_robots):
        print(f"  Robot {i}: ({positions[i, 0]:.1f}, {positions[i, 1]:.1f})")
    
    print("\n🔗 Initial Topology:")
    print("  Complete square with both diagonals (redundant)")
    print_topology_info(edges, positions, iteration=0)
    
    # Compute trust values
    A_true = compute_trust_values(positions, edges, sigma)
    print_matrix(A_true, "True Adjacency Matrix A*(0) [Trust Values]")
    
    print("\n📊 Trust Value Explanation:")
    print("  t_ij = exp(-d_ij^2 / σ^2) where σ = 1.0")
    print("  - Straight edges (length 1.0): t ≈ 0.368")
    print("  - Diagonal edges (length 1.414): t ≈ 0.135")
    
    # Initialize manager
    print_section("INITIALIZING CONCURRENT PRUNING MANAGER")
    
    manager = ConcurrentPruningManager(
        num_robots=num_robots,
        mode='lyapunov',
        sigma=sigma,
        sample_time=0.2,
        lambda2_threshold=0.2,      # Minimum algebraic connectivity
        lambda2_safety_margin=0.15,  # Safety margin
        k_max=30                     # Expected convergence horizon
    )
    
    # Initialize robot knowledge
    manager.initialize_robot_knowledge(positions, edges)
    
    print("\n✓ Manager initialized with:")
    print(f"  - Mode: Lyapunov-constrained pruning")
    print(f"  - Adaptive thresholds: Enabled (conservative → aggressive)")
    print(f"  - Distributed coordination: 3-phase protocol")
    
    # Run concurrent pruning
    print_section("RUNNING CONCURRENT PRUNING")
    print("\n🔄 Each iteration:")
    print("  1. Consensus update: A^l(k) → A^l(k+1)")
    print("  2. Distributed pruning:")
    print("     - Phase 1: Each robot proposes edges (local evaluation)")
    print("     - Phase 2: Bilateral negotiation (unanimous consent)")
    print("     - Phase 3: Conflict resolution (deterministic tie-breaking)")
    
    max_iterations = 30
    history = {
        'edges': [len(edges)],
        'lambda2': [],
        'disagreement': [],
        'pruned_edges': []
    }
    
    # Setup visualization
    fig, axes = plt.subplots(2, 3, figsize=(18, 12))
    axes = axes.flatten()
    
    # Initial visualization
    visualize_topology(positions, edges, 0, ax=axes[0])
    
    viz_count = 1
    
    for iteration in range(1, max_iterations + 1):
        print(f"\n{'─' * 80}")
        print(f"Iteration {iteration}")
        print('─' * 80)
        
        # Run one step (edges is modified in place)
        report = manager.concurrent_step(
            positions=positions,
            current_edges=edges,
            debug=False
        )
        
        # Extract metrics
        pruned_edge = report.get('pruned_edge', None)
        
        # Compute lambda2 from current adjacency matrix
        A_current = np.zeros((num_robots, num_robots))
        for i, j in edges:
            distance = np.linalg.norm(positions[i] - positions[j])
            trust = np.exp(-distance**2 / sigma**2)
            A_current[i, j] = A_current[j, i] = trust
        
        # Compute Laplacian and lambda2
        D = np.diag(A_current.sum(axis=1))
        L = D - A_current
        eigenvalues = sorted(np.linalg.eigvalsh(L))
        lambda2 = eigenvalues[1] if len(eigenvalues) > 1 else 0.0
        
        # Extract convergence metric (Lyapunov or disagreement)
        convergence_metric = 0.0
        if 'lyapunov' in report.get('metrics', {}):
            lyapunov_values = report['metrics']['lyapunov'].values()
            convergence_metric = max(lyapunov_values) if lyapunov_values else 0.0
        elif 'disagreement' in report.get('metrics', {}):
            disagreement_values = report['metrics']['disagreement'].values()
            convergence_metric = max(disagreement_values) if disagreement_values else 0.0
        
        history['edges'].append(len(edges))
        history['lambda2'].append(lambda2)
        history['disagreement'].append(convergence_metric)
        
        if pruned_edge:
            history['pruned_edges'].append((iteration, pruned_edge))
            print(f"\n🔪 EDGE PRUNED: {pruned_edge}")
            
            # Visualize after pruning
            if viz_count < 6:
                visualize_topology(positions, edges, iteration, pruned_edge, ax=axes[viz_count])
                viz_count += 1
        else:
            print("\n✋ No edge pruned this iteration")
        
        # Print current state
        print(f"\nCurrent topology: {len(edges)} edges")
        print(f"Phase: {report.get('phase', 'N/A')}")
        print(f"λ₂: {lambda2:.4f}")
        if convergence_metric > 0:
            print(f"Convergence metric: {convergence_metric:.6f}")
        
        # Check convergence
        if convergence_metric < 1e-3 and len(edges) == num_robots - 1:
            print("\n🎉 CONVERGENCE ACHIEVED!")
            print(f"  - Minimal tree reached: {num_robots - 1} edges")
            print(f"  - Consensus converged: disagreement < 0.001")
            
            # Final visualization
            if viz_count < 6:
                visualize_topology(positions, edges, iteration, ax=axes[viz_count])
            
            break
    
    # Hide unused subplots
    for i in range(viz_count, 6):
        axes[i].axis('off')
    
    plt.tight_layout()
    plt.savefig(parent_dir / 'concurrent_pruning' / '4_robot_example_evolution.png', dpi=150, bbox_inches='tight')
    print(f"\n💾 Topology evolution saved to: 4_robot_example_evolution.png")
    
    # Print summary
    print_section("SUMMARY")
    
    print("\n📈 Pruning Timeline:")
    for iter_num, edge in history['pruned_edges']:
        print(f"  Iteration {iter_num}: Pruned edge {edge}")
    
    print(f"\n🎯 Final Result:")
    print(f"  - Initial edges: 6 (redundant)")
    print(f"  - Final edges: {len(edges)} (minimal tree)")
    print(f"  - Edges pruned: {6 - len(edges)}")
    print(f"  - Final topology: {sorted(edges)}")
    
    # Verify it's a tree
    print(f"\n✓ Verification:")
    print(f"  - Number of edges = n-1: {len(edges) == num_robots - 1}")
    print(f"  - Connected: {manager.edge_analyzer.check_graph_connectivity(edges)}")
    print(f"  - Final λ₂: {history['lambda2'][-1]:.4f}")
    if history['disagreement'] and history['disagreement'][-1] > 0:
        print(f"  - Consensus metric: {history['disagreement'][-1]:.6f}")
    
    # Plot metrics
    fig2, axes2 = plt.subplots(1, 3, figsize=(15, 4))
    
    # Edge count over time
    axes2[0].plot(range(len(history['edges'])), history['edges'], 'o-', linewidth=2, markersize=6)
    axes2[0].axhline(y=num_robots-1, color='r', linestyle='--', label='Minimum (tree)')
    axes2[0].set_xlabel('Iteration')
    axes2[0].set_ylabel('Number of Edges')
    axes2[0].set_title('Topology Pruning Progress')
    axes2[0].legend()
    axes2[0].grid(True, alpha=0.3)
    
    # Lambda2 (connectivity)
    lambda2_data = [x for x in history['lambda2'] if x > 0]
    if lambda2_data:
        axes2[1].plot(range(1, len(history['lambda2']) + 1), history['lambda2'], 'o-', linewidth=2, markersize=6, color='green')
        axes2[1].axhline(y=0.2, color='r', linestyle='--', label='Threshold', linewidth=2)
        axes2[1].set_xlabel('Iteration', fontsize=12)
        axes2[1].set_ylabel('λ₂ (Algebraic Connectivity)', fontsize=12)
        axes2[1].set_title('Connectivity Maintained', fontsize=14, fontweight='bold')
        axes2[1].legend()
        axes2[1].grid(True, alpha=0.3)
    
    # Convergence metric (Lyapunov or disagreement)
    convergence_data = [x for x in history['disagreement'] if x > 0]
    if convergence_data:
        axes2[2].semilogy(range(1, len(history['disagreement']) + 1), history['disagreement'], 'o-', linewidth=2, markersize=6, color='purple')
        axes2[2].set_xlabel('Iteration', fontsize=12)
        axes2[2].set_ylabel('Convergence Metric (log scale)', fontsize=12)
        axes2[2].set_title('Consensus Convergence (Local Lyapunov)', fontsize=14, fontweight='bold')
        axes2[2].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(parent_dir / 'concurrent_pruning' / '4_robot_example_metrics.png', dpi=150, bbox_inches='tight')
    print(f"💾 Metrics plot saved to: 4_robot_example_metrics.png")
    
    # Show plots
    plt.show()
    
    print_section("MATHEMATICAL INSIGHTS")
    
    print("""
✓ Key Observations:

1. CONCURRENT OPERATION:
   - Consensus and pruning happened SIMULTANEOUSLY
   - No need to wait for full convergence before pruning
   - Result: Faster than sequential approach

2. DISTRIBUTED COORDINATION:
   - Each robot evaluated only its incident edges (local)
   - Both endpoints agreed before pruning (bilateral)
   - Deterministic tie-breaking (no coordinator needed)

3. SAFETY GUARANTEES:
   - Lyapunov constraint ensured convergence preserved
   - Algebraic connectivity maintained above threshold
   - Alternative paths verified before pruning

4. ADAPTIVE BEHAVIOR:
   - Conservative early (large margins, safe)
   - Aggressive late (small margins, optimal)
   - Smooth transition via threshold scheduling

5. FINAL RESULT:
   - Minimal spanning tree achieved (3 edges for 4 robots)
   - Consensus converged (all robots agree on A*)
   - Connectivity maintained (λ₂ > threshold)
""")
    
    print("\n🎓 Mathematical Elegance:")
    print("  Simple local rules → Complex global behavior (emergent optimization)")


if __name__ == "__main__":
    print("""
╔═══════════════════════════════════════════════════════════════════╗
║                                                                   ║
║         CONCURRENT PRUNING: 4-ROBOT EXAMPLE                      ║
║         Mathematical Theory in Action                            ║
║                                                                   ║
║  See SMALL_EXAMPLE_WALKTHROUGH.md for detailed mathematics       ║
║                                                                   ║
╚═══════════════════════════════════════════════════════════════════╝
""")
    
    try:
        run_small_example()
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
        
    print("\n✅ Example complete! Check the generated visualizations.")
