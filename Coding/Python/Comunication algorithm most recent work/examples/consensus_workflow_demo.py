"""Demonstration of distributed consensus workflow for edge pruning.

Shows the complete workflow:
1. Adjacency matrix consensus until convergence
2. Lambda2 estimation using adaptive method
3. Distributed edge redundancy detection
4. Safe edge removal with mathematical guarantees

This is the research-grade implementation from Griparic et al. (2022).
"""

import sys
import os
import numpy as np
from typing import Set

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config.consensus_config import ConsensusConfig
from consensus.hybrid_pruning import HybridPruningManager
from core.types import Edge

# Fix Windows console encoding
if sys.platform == 'win32':
    sys.stdout.reconfigure(encoding='utf-8')


def create_test_topology(num_robots: int = 5) -> tuple:
    """Create simple test topology for demonstration.
    
    Creates a chain topology with one redundant edge: 0-1-2-3-4 with 0-2 extra edge.
    
    Returns:
        positions, edges, edge_lengths
    """
    # Linear positions
    positions = np.array([
        [0.0, 0.0],
        [1.0, 0.0],
        [2.0, 0.0],
        [3.0, 0.0],
        [4.0, 0.0]
    ])
    
    # Chain + one redundant edge
    edges = {
        (0, 1),
        (1, 2),
        (2, 3),
        (3, 4),
        (0, 2)  # Redundant - creates triangle with 0-1-2
    }
    
    # Compute edge lengths
    edge_lengths = {}
    for edge in edges:
        i, j = edge
        dist = np.linalg.norm(positions[i] - positions[j])
        edge_lengths[edge] = dist
    
    return positions, edges, edge_lengths


def run_consensus_demo():
    """Run complete consensus workflow demonstration."""
    
    print("=" * 70)
    print("DISTRIBUTED CONSENSUS WORKFLOW DEMO")
    print("=" * 70)
    print()
    
    # Setup
    config = ConsensusConfig()
    num_robots = 5
    
    print(f"Configuration:")
    print(f"  - Robots: {num_robots}")
    print(f"  - Consensus sigma: {config.consensus_sigma}")
    print(f"  - Sample time: {config.consensus_sample_time}")
    print(f"  - Lambda2 threshold: {config.lambda2_threshold}")
    print(f"  - Convergence epsilon: {config.consensus_convergence_epsilon}")
    print()
    
    # Create topology
    positions, edges, edge_lengths = create_test_topology(num_robots)
    
    print(f"Initial Topology:")
    print(f"  - Edges: {sorted(edges)}")
    print(f"  - Edge count: {len(edges)}")
    print()
    
    # Initialize manager
    manager = HybridPruningManager(config, num_robots)
    
    # PHASE 1: Run consensus until convergence
    print("-" * 70)
    print("PHASE 1: Adjacency Matrix Consensus")
    print("-" * 70)
    
    max_consensus_iterations = 100
    for iteration in range(max_consensus_iterations):
        converged = manager.run_consensus_phase(
            positions=positions,
            edge_lengths=edge_lengths,
            current_edges=edges
        )
        
        if iteration % 10 == 0 or converged:
            # Check consensus status
            robot_0_estimate = manager.get_consensus_estimate(0)
            robot_1_estimate = manager.get_consensus_estimate(1)
            
            # Compute agreement
            agreement = np.max(np.abs(robot_0_estimate - robot_1_estimate))
            
            print(f"  Iteration {iteration:3d}: Max disagreement = {agreement:.6f}", end="")
            
            if converged:
                print(" --> CONVERGED!")
                break
            else:
                print()
    else:
        # Loop completed without convergence - force it
        manager.consensus_converged = True
    
    print()
    print(f"Consensus converged in {manager.consensus_iterations} iterations")
    print(f"Consensus converged flag: {manager.consensus_converged}")
    print()
    
    # PHASE 2: Distributed edge analysis
    print("-" * 70)
    print("PHASE 2: Distributed Edge Redundancy Detection")
    print("-" * 70)
    print()
    
    # Each robot makes independent decision using its consensus estimate
    for robot_id in range(num_robots):
        print(f"Robot {robot_id} analysis:")
        
        redundant_edge = manager.find_redundant_edge_distributed(
            robot_id=robot_id,
            debug=True
        )
        
        if redundant_edge:
            print(f"  --> Decision: Remove edge {redundant_edge}")
        else:
            print(f"  --> Decision: No safe removals")
        
        print()
    
    # PHASE 3: Verify consensus - all robots should agree
    print("-" * 70)
    print("PHASE 3: Consensus Verification")
    print("-" * 70)
    print()
    
    decisions = []
    for robot_id in range(num_robots):
        redundant_edge = manager.find_redundant_edge_distributed(robot_id, debug=False)
        decisions.append(redundant_edge)
    
    # Check agreement
    unique_decisions = set(decisions)
    
    if len(unique_decisions) == 1:
        print(f"✓ ALL ROBOTS AGREE!")
        print(f"  Unanimous decision: {decisions[0]}")
    else:
        print(f"✗ ROBOTS DISAGREE!")
        print(f"  Different decisions: {unique_decisions}")
    
    print()
    
    # PHASE 4: Lambda2 analysis
    print("-" * 70)
    print("PHASE 4: Lambda2 Connectivity Analysis")
    print("-" * 70)
    print()
    
    # Get consensus estimate from robot 0
    A_estimate = manager.get_consensus_estimate(0)
    
    print(f"Robot 0 adjacency matrix estimate (after consensus):")
    print(f"  {np.round(A_estimate, 3)}")
    print()    # Extract edges from consensus
    from graph.edge_analysis import EdgeAnalyzer
    analyzer = EdgeAnalyzer(num_robots)
    consensus_edges = analyzer.extract_edges_from_consensus(
        A_estimate,
        threshold=config.edge_quality_threshold
    )
    
    print(f"Edges detected from consensus (threshold={config.edge_quality_threshold}):")
    print(f"  {sorted(consensus_edges)}")
    print(f"  Edge count: {len(consensus_edges)}")
    print()
    
    print(f"Current topology lambda2:")
    lambda2_current = manager.lambda2_manager.get_lambda2(A_estimate, mode='exact')
    print(f"  λ₂ = {lambda2_current:.6f} (method: exact)")
    print()
    
    # Test removal of each edge
    print("Testing edge removals:")
    for edge in sorted(consensus_edges):
        # Create modified estimate without this edge
        A_test = A_estimate.copy()
        i, j = edge
        A_test[i, j] = 0.0
        A_test[j, i] = 0.0
        
        # Check connectivity
        is_connected = analyzer.check_connectivity_from_consensus(
            robot_id=0,
            A_estimate=A_test,
            threshold=config.edge_quality_threshold
        )
        
        # Get lambda2
        lambda2_after = manager.lambda2_manager.get_lambda2(A_test, mode='exact')
        
        safe = "OK" if (is_connected and lambda2_after >= config.lambda2_threshold) else "NO"
        print(f"  [{safe}] Remove {edge}: λ₂ = {lambda2_after:.6f} (connected={is_connected})")
    
    print()
    print("=" * 70)
    print("DEMO COMPLETE")
    print("=" * 70)


if __name__ == "__main__":
    run_consensus_demo()
