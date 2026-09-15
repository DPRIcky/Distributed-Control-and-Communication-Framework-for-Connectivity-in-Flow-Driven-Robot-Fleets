"""Unit tests for adjacency matrix consensus protocol.

Tests verify that the consensus algorithm from Griparic et al. (2022) correctly
converges to the true topology for various graph configurations.
"""

import numpy as np

from consensus.adjacency_consensus import AdjacencyMatrixConsensus


def test_consensus_simple_topology():
    """Test consensus convergence on simple 5-robot topology."""
    num_robots = 5
    consensus = AdjacencyMatrixConsensus(
        num_robots=num_robots,
        sigma=1.0,
        sample_time=0.2,
        convergence_epsilon=1e-3
    )
    
    # Define true topology: chain with extra edge
    # 0 -- 1 -- 2 -- 3 -- 4
    #      |         |
    #       ----------
    true_topology = {
        0: {1: 1.0},
        1: {0: 1.0, 2: 0.8, 3: 1.2},
        2: {1: 0.8, 3: 0.9},
        3: {1: 1.2, 2: 0.9, 4: 1.1},
        4: {3: 1.1}
    }
    
    # Initialize each robot with local knowledge
    for robot_id in range(num_robots):
        neighbors = set(true_topology.get(robot_id, {}).keys())
        distances = true_topology.get(robot_id, {})
        consensus.initialize_robot_knowledge(robot_id, neighbors, distances)
    
    # Run consensus updates
    max_rounds = 100
    for round_num in range(max_rounds):
        # Each robot updates
        for robot_id in range(num_robots):
            neighbors = set(true_topology.get(robot_id, {}).keys())
            
            # Direct observations (only for incident edges)
            observations = {}
            for neighbor_id in neighbors:
                distance = true_topology[robot_id][neighbor_id]
                observations[(robot_id, neighbor_id)] = distance
                observations[(neighbor_id, robot_id)] = distance
            
            consensus.consensus_update_step(robot_id, neighbors, observations)
        
        # Check convergence
        if consensus.check_convergence():
            print(f"Consensus converged at round {round_num}")
            break
    
    # Verify convergence
    assert consensus.consensus_converged, "Consensus should have converged"
    
    # Verify all robots have similar estimates
    estimates = [consensus.get_adjacency_estimate(i) for i in range(num_robots)]
    reference = estimates[0]
    
    for i, A in enumerate(estimates[1:], 1):
        diff = np.max(np.abs(A - reference))
        assert diff < 0.01, f"Robot {i} estimate differs from robot 0 by {diff}"
    
    print(f"✓ All robots converged to same estimate")
    
    # Verify estimate is close to true topology
    # Check that edges exist where they should
    A = reference
    assert A[0, 1] > 0.5 and A[1, 0] > 0.5, "Edge (0,1) should exist"
    assert A[1, 2] > 0.5 and A[2, 1] > 0.5, "Edge (1,2) should exist"
    assert A[2, 3] > 0.5 and A[3, 2] > 0.5, "Edge (2,3) should exist"
    assert A[3, 4] > 0.5 and A[4, 3] > 0.5, "Edge (3,4) should exist"
    assert A[1, 3] > 0.5 and A[3, 1] > 0.5, "Edge (1,3) should exist"
    
    # Check that non-edges are weak
    assert A[0, 2] < 0.3, "Edge (0,2) should not exist"
    assert A[0, 4] < 0.3, "Edge (0,4) should not exist"
    
    print(f"✓ Consensus estimate matches true topology")


def test_consensus_status():
    """Test consensus status tracking."""
    num_robots = 3
    consensus = AdjacencyMatrixConsensus(num_robots=num_robots)
    
    status = consensus.get_consensus_status()
    assert status['converged'] == False
    assert status['round'] == 0
    
    print(f"✓ Consensus status tracking works")


def test_trust_function():
    """Test Gaussian trust function properties."""
    consensus = AdjacencyMatrixConsensus(num_robots=5, sigma=1.0)
    
    # Perfect link (distance=0) should have quality=1
    assert abs(consensus._trust_function(0.0) - 1.0) < 0.01
    
    # Far link should have low quality
    assert consensus._trust_function(5.0) < 0.01
    
    # Mid-range link
    quality_1 = consensus._trust_function(1.0)
    assert 0.3 < quality_1 < 0.5
    
    print(f"✓ Trust function behaves correctly")


if __name__ == "__main__":
    print("Running consensus convergence tests...\n")
    
    test_trust_function()
    test_consensus_status()
    test_consensus_simple_topology()
    
    print("\n✅ All tests passed!")
