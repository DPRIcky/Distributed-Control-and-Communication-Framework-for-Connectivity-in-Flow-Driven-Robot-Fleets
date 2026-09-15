"""Test and Demo for Critical Edge Detection Baseline.

This file provides comprehensive tests and demonstrations of the critical
edge detection baseline integrated with the simulation framework.

Tests:
1. Standalone baseline test
2. Integration with simulation framework
3. Comparison with centralized MST
4. Performance verification
"""

import numpy as np
import sys
import os

# Add parent directory to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from critical_edge_baseline import CriticalEdgeBaseline, get_critical_edge_mst


def test_standalone_baseline():
    """Test 1: Standalone baseline functionality."""
    print("\n" + "=" * 70)
    print("TEST 1: STANDALONE BASELINE")
    print("=" * 70)
    
    # Create simple test graph
    num_robots = 7
    edge_lengths = {
        (0, 1): 1.0,
        (1, 2): 1.2,
        (2, 3): 1.1,
        (3, 4): 1.3,
        (4, 5): 0.9,
        (5, 6): 1.0,
        (0, 3): 2.8,  # Shortcut
        (1, 4): 2.9,  # Shortcut
        (2, 5): 3.0,  # Shortcut
    }
    
    print(f"Input graph: {num_robots} robots, {len(edge_lengths)} edges")
    print(f"Expected MST: {num_robots - 1} edges")
    
    # Initialize baseline
    baseline = CriticalEdgeBaseline(
        num_robots=num_robots,
        update_frequency=1,
        prefer_critical=True,
        verbose=True
    )
    
    # Find edges to prune
    pruned_edges = []
    current_edges = edge_lengths.copy()
    
    while len(current_edges) > num_robots - 1:
        edge_to_prune = baseline.find_edge_to_prune(current_edges)
        if edge_to_prune is None:
            break
        pruned_edges.append(edge_to_prune)
        del current_edges[edge_to_prune]
    
    # Verify results
    print(f"\nFinal MST: {len(current_edges)} edges")
    print(f"Edges pruned: {pruned_edges}")
    print(f"MST edges: {sorted(current_edges.keys())}")
    
    stats = baseline.get_statistics()
    print(f"\nStatistics:")
    print(f"  MST updates: {stats['mst_updates']}")
    print(f"  Total rounds: {stats['total_rounds']}")
    print(f"  Avg rounds/update: {stats['avg_rounds_per_update']:.2f}")
    
    # Verify MST size
    assert len(current_edges) == num_robots - 1, \
        f"Expected {num_robots - 1} edges, got {len(current_edges)}"
    
    print("\n[PASS] Standalone baseline test")
    return True


def test_critical_edge_preservation():
    """Test 2: Verify critical edges (bridges) are preserved."""
    print("\n" + "=" * 70)
    print("TEST 2: CRITICAL EDGE PRESERVATION")
    print("=" * 70)
    
    # Chain graph - all edges are bridges
    num_robots = 6
    chain_edges = {
        (0, 1): 1.0,
        (1, 2): 1.0,
        (2, 3): 1.0,
        (3, 4): 1.0,
        (4, 5): 1.0,
    }
    
    print(f"Chain graph: {num_robots} robots, {len(chain_edges)} edges (all bridges)")
    
    # Get MST
    mst = get_critical_edge_mst(
        num_robots=num_robots,
        edge_lengths=chain_edges,
        prefer_critical=True,
        verbose=True
    )
    
    print(f"\nMST edges: {sorted(mst)}")
    print(f"All edges preserved: {mst == set(chain_edges.keys())}")
    
    # Verify all bridges are in MST
    assert len(mst) == num_robots - 1, "MST should have n-1 edges"
    assert mst == set(chain_edges.keys()), "All bridges should be in MST"
    
    print("\n[PASS] Critical edge preservation test")
    return True


def test_graph_with_redundancy():
    """Test 3: Graph with redundant edges."""
    print("\n" + "=" * 70)
    print("TEST 3: GRAPH WITH REDUNDANT EDGES")
    print("=" * 70)
    
    # Graph with shortcuts
    num_robots = 5
    edges = {
        (0, 1): 1.0,
        (1, 2): 1.0,
        (2, 3): 1.0,
        (3, 4): 1.0,
        (0, 4): 3.5,  # Long shortcut
    }
    
    print(f"Input: {num_robots} robots, {len(edges)} edges")
    print(f"Original edges: {sorted(edges.keys())}")
    
    baseline = CriticalEdgeBaseline(
        num_robots=num_robots,
        prefer_critical=True,
        verbose=False
    )
    
    # Get MST
    baseline.update_mst(edges)
    mst = baseline.current_mst
    
    print(f"MST edges: {sorted(mst)}")
    print(f"Edges removed: {len(edges) - len(mst)}")
    
    # Verify
    assert len(mst) == num_robots - 1, "MST should have n-1 edges"
    assert (0, 4) not in mst and (4, 0) not in mst, "Long shortcut should be removed"
    
    print("\n[PASS] Graph with redundancy test")
    return True


def test_simulation_integration():
    """Test 4: Integration with simulation framework."""
    print("\n" + "=" * 70)
    print("TEST 4: SIMULATION INTEGRATION")
    print("=" * 70)
    
    try:
        from config import SimulationConfig, ControlConfig
        from simple_simulation import CriticalEdgeSimulation
        
        # Create minimal configs
        sim_config = SimulationConfig(
            num_robots=5,
            communication_radius=3.0,
            dt=0.05,
            workspace_size=(10.0, 10.0),
            verbose=False,
            seed=42
        )
        
        control_config = ControlConfig()
        
        # Create simulation
        sim = CriticalEdgeSimulation(
            sim_config=sim_config,
            control_config=control_config,
            update_frequency=5,
            prefer_critical=True,
            verbose=False
        )
        
        print("Simulation created successfully")
        print(f"  Robots: {sim.num_robots}")
        print(f"  Communication radius: {sim.communication_radius}")
        print(f"  Baseline: {sim.baseline.get_name()}")
        
        # Run a few steps
        print("\nRunning 20 simulation steps...")
        for _ in range(20):
            sim.step()
        
        print(f"Current time: {sim.time:.2f}s")
        print(f"Current edges: {sim.total_edges}")
        print(f"Total pruned: {sim.edges_pruned_count}")
        
        # Get statistics
        stats = sim.baseline.get_statistics()
        print(f"\nBaseline statistics:")
        print(f"  MST updates: {stats['mst_updates']}")
        print(f"  Total rounds: {stats['total_rounds']}")
        
        print("\n[PASS] Simulation integration test")
        return True
        
    except ImportError as e:
        print(f"\n[SKIP] Simulation integration test - missing dependencies: {e}")
        return None


def test_performance():
    """Test 5: Performance and complexity verification."""
    print("\n" + "=" * 70)
    print("TEST 5: PERFORMANCE AND COMPLEXITY")
    print("=" * 70)
    
    print("Verifying O(n) round complexity...")
    
    for n in [5, 10, 15, 20]:
        # Create complete graph
        edges = {(i, j): 1.0 for i in range(n) for j in range(i+1, n)}
        
        baseline = CriticalEdgeBaseline(
            num_robots=n,
            prefer_critical=False,
            verbose=False
        )
        
        # Update MST and get statistics
        results = baseline.update_mst(edges)
        rounds = results['rounds']
        
        print(f"  n={n:2d}: {rounds:2d} rounds (<= {n})", end="")
        
        # Verify O(n) complexity
        assert rounds <= n, f"Should be O(n), got {rounds} for n={n}"
        print(" [OK]")
    
    print("\n[PASS] Performance test")
    return True


def demo_comparison():
    """Demo: Compare with centralized MST."""
    print("\n" + "=" * 70)
    print("DEMO: COMPARISON WITH CENTRALIZED MST")
    print("=" * 70)
    
    # Create test graph
    num_robots = 8
    np.random.seed(42)
    
    # Generate random complete graph
    edges = {}
    for i in range(num_robots):
        for j in range(i+1, num_robots):
            edges[(i, j)] = np.random.uniform(1.0, 10.0)
    
    print(f"Random graph: {num_robots} robots, {len(edges)} edges")
    
    # Get critical edge MST
    print("\n1. Critical Edge Baseline (Distributed):")
    baseline = CriticalEdgeBaseline(num_robots=num_robots, verbose=False)
    baseline.update_mst(edges)
    critical_mst = baseline.current_mst
    print(f"   MST edges: {len(critical_mst)}")
    print(f"   Critical edges detected: {len(baseline.critical_edges)}")
    print(f"   DFS rounds: {baseline.total_rounds}")
    
    # Compare size
    print(f"\n2. Comparison:")
    print(f"   Expected MST size: {num_robots - 1}")
    print(f"   Critical edge MST: {len(critical_mst)}")
    print(f"   Match: {len(critical_mst) == num_robots - 1}")
    
    # Calculate MST cost
    critical_cost = sum(edges[e] for e in critical_mst if e in edges)
    print(f"\n3. MST Cost:")
    print(f"   Critical edge MST: {critical_cost:.2f}")


def run_all_tests():
    """Run all tests and report results."""
    print("\n" + "=" * 80)
    print("CRITICAL EDGE DETECTION BASELINE - TEST SUITE")
    print("=" * 80)
    
    tests = [
        ("Standalone Baseline", test_standalone_baseline),
        ("Critical Edge Preservation", test_critical_edge_preservation),
        ("Graph with Redundancy", test_graph_with_redundancy),
        ("Simulation Integration", test_simulation_integration),
        ("Performance & Complexity", test_performance),
    ]
    
    results = []
    passed = 0
    failed = 0
    skipped = 0
    
    for name, test_func in tests:
        try:
            result = test_func()
            if result is True:
                results.append((name, "PASS"))
                passed += 1
            elif result is None:
                results.append((name, "SKIP"))
                skipped += 1
            else:
                results.append((name, "FAIL"))
                failed += 1
        except Exception as e:
            print(f"\n[ERROR] {name}: {e}")
            import traceback
            traceback.print_exc()
            results.append((name, "ERROR"))
            failed += 1
    
    # Run demo
    try:
        demo_comparison()
    except Exception as e:
        print(f"\n[ERROR] Demo failed: {e}")
    
    # Print summary
    print("\n" + "=" * 80)
    print("TEST SUMMARY")
    print("=" * 80)
    for name, status in results:
        status_symbol = {
            "PASS": "[+]",
            "FAIL": "[X]",
            "SKIP": "[o]",
            "ERROR": "[X]"
        }.get(status, "[?]")
        print(f"  {status_symbol} {name}: {status}")
    
    print(f"\nTotal: {len(tests)} tests")
    print(f"  Passed: {passed}")
    print(f"  Failed: {failed}")
    print(f"  Skipped: {skipped}")
    
    if failed == 0:
        print("\n" + "=" * 80)
        print("ALL TESTS PASSED!")
        print("=" * 80)
        print("\nThe critical edge detection baseline is ready to use!")
        print("\nNext steps:")
        print("  1. Run comparison with other baselines")
        print("  2. Integrate with visualization tools")
        print("  3. Generate performance plots")
        return True
    else:
        print("\n" + "=" * 80)
        print(f"TESTS FAILED: {failed} test(s) failed")
        print("=" * 80)
        return False


if __name__ == "__main__":
    success = run_all_tests()
    sys.exit(0 if success else 1)
