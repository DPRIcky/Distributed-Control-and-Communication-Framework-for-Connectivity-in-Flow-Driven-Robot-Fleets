"""Unit tests for adaptive lambda2 estimation."""

import sys
import numpy as np

# Add parent directory to path
sys.path.insert(0, '.')

from graph.lambda2_estimator import AdaptiveLambda2Manager


def test_exact_computation():
    """Test exact lambda2 computation on known topology."""
    print("\n" + "="*70)
    print("TEST 1: Exact Lambda2 Computation")
    print("="*70)
    
    # Create simple chain: 0-1-2-3-4
    n = 5
    A = np.array([
        [0, 1, 0, 0, 0],
        [1, 0, 1, 0, 0],
        [0, 1, 0, 1, 0],
        [0, 0, 1, 0, 1],
        [0, 0, 0, 1, 0]
    ], dtype=float)
    
    manager = AdaptiveLambda2Manager(num_robots=n)
    
    lambda2 = manager.get_lambda2(A, mode='exact')
    
    print(f"Chain topology (0-1-2-3-4):")
    print(f"  λ₂ = {lambda2:.6f}")
    
    # Known value for chain of 5 nodes
    expected = 2 * (1 - np.cos(np.pi / 5))  # Approx 0.382
    error = abs(lambda2 - expected)
    
    print(f"  Expected: {expected:.6f}")
    print(f"  Error: {error:.6f}")
    
    if error < 0.01:
        print("  ✓ PASS: Lambda2 matches expected value")
        return True
    else:
        print("  ✗ FAIL: Lambda2 does not match")
        return False


def test_incremental_update():
    """Test incremental update after edge removal."""
    print("\n" + "="*70)
    print("TEST 2: Incremental Lambda2 Update")
    print("="*70)
    
    # Start with triangle + extra edges
    n = 5
    A_initial = np.array([
        [0, 1, 1, 0, 0],
        [1, 0, 1, 0, 0],
        [1, 1, 0, 1, 0],
        [0, 0, 1, 0, 1],
        [0, 0, 0, 1, 0]
    ], dtype=float)
    
    manager = AdaptiveLambda2Manager(num_robots=n)
    
    # Get initial lambda2 (exact)
    lambda2_initial = manager.get_lambda2(A_initial, mode='exact')
    print(f"Initial topology:")
    print(f"  λ₂ = {lambda2_initial:.6f}")
    
    # Remove edge (0,2) - the redundant triangle edge
    A_after = A_initial.copy()
    A_after[0, 2] = 0
    A_after[2, 0] = 0
    
    # Use incremental update
    edge_removed = (0, 2)
    lambda2_incremental = manager.update_after_edge_removal(edge_removed)
    
    print(f"\nAfter removing edge (0,2) - incremental:")
    print(f"  λ₂ = {lambda2_incremental:.6f}")
    
    # Compute exact for comparison
    manager_exact = AdaptiveLambda2Manager(num_robots=n)
    lambda2_exact = manager_exact.get_lambda2(A_after, mode='exact')
    
    print(f"\nExact computation:")
    print(f"  λ₂ = {lambda2_exact:.6f}")
    
    error = abs(lambda2_incremental - lambda2_exact)
    print(f"\nIncremental error: {error:.6f}")
    
    if error < 0.05:
        print("  ✓ PASS: Incremental update within tolerance")
        return True
    else:
        print("  ✗ FAIL: Incremental error too large")
        return False


def test_safety_verification():
    """Test three-tier safety verification for edge removal."""
    print("\n" + "="*70)
    print("TEST 3: Safety Verification for Edge Removal")
    print("="*70)
    
    # Create topology with one safe and one unsafe removal
    n = 5
    A = np.array([
        [0, 1, 1, 0, 0],
        [1, 0, 1, 0, 0],
        [1, 1, 0, 1, 0],
        [0, 0, 1, 0, 1],
        [0, 0, 0, 1, 0]
    ], dtype=float)
    
    manager = AdaptiveLambda2Manager(
        num_robots=n,
        lambda2_threshold=0.1,
        safety_margin=0.05
    )
    
    # Initialize with exact computation
    lambda2_initial = manager.get_lambda2(A, mode='exact')
    print(f"Initial λ₂ = {lambda2_initial:.6f}")
    
    # Test safe removal (0,2)
    print(f"\nTest 1: Remove (0,2) - should be SAFE")
    safe, lambda2_after = manager.verify_removal_safety(A, (0, 2))
    print(f"  Safe: {safe}, λ₂ after: {lambda2_after:.6f}")
    
    if safe and lambda2_after > manager.lambda2_min:
        print("  ✓ PASS: Correctly identified safe removal")
        test1_pass = True
    else:
        print("  ✗ FAIL: Should be safe")
        test1_pass = False
    
    # Test unsafe removal (3,4) - breaks connectivity
    print(f"\nTest 2: Remove (3,4) - should be UNSAFE")
    safe, lambda2_after = manager.verify_removal_safety(A, (3, 4))
    print(f"  Safe: {safe}, λ₂ after: {lambda2_after:.6f}")
    
    if not safe or lambda2_after < manager.lambda2_min:
        print("  ✓ PASS: Correctly identified unsafe removal")
        test2_pass = True
    else:
        print("  ✗ FAIL: Should be unsafe")
        test2_pass = False
    
    return test1_pass and test2_pass


def test_adaptive_mode_selection():
    """Test automatic mode selection based on state."""
    print("\n" + "="*70)
    print("TEST 4: Adaptive Mode Selection")
    print("="*70)
    
    n = 5
    A = np.array([
        [0, 1, 1, 0, 0],
        [1, 0, 1, 0, 0],
        [1, 1, 0, 1, 0],
        [0, 0, 1, 0, 1],
        [0, 0, 0, 1, 0]
    ], dtype=float)
    
    manager = AdaptiveLambda2Manager(
        num_robots=n,
        max_incremental_updates=3
    )
    
    # First call should use exact (no cache)
    print("Call 1 (no cache):")
    lambda2_1 = manager.get_lambda2(A, mode='auto')
    print(f"  λ₂ = {lambda2_1:.6f}, cache exists: {manager.lambda2_cache is not None}")
    
    # Second call should use cache
    print("\nCall 2 (with cache):")
    lambda2_2 = manager.get_lambda2(A, mode='auto')
    print(f"  λ₂ = {lambda2_2:.6f}, cache reused: {lambda2_2 == lambda2_1}")
    
    # After max incremental updates, should recompute
    print(f"\nAfter {manager.max_incremental_updates} incremental updates:")
    for i in range(manager.max_incremental_updates):
        manager.incremental_count += 1
    
    lambda2_3 = manager.get_lambda2(A, mode='auto')
    print(f"  λ₂ = {lambda2_3:.6f}, incremental_count reset: {manager.incremental_count}")
    
    if manager.incremental_count == 0:
        print("  ✓ PASS: Correctly triggered exact recomputation")
        return True
    else:
        print("  ✗ FAIL: Should have reset counter")
        return False


def run_all_tests():
    """Run all lambda2 estimator tests."""
    print("\n" + "#"*70)
    print("# LAMBDA2 ESTIMATOR TEST SUITE")
    print("#"*70)
    
    results = []
    
    results.append(("Exact Computation", test_exact_computation()))
    results.append(("Incremental Update", test_incremental_update()))
    results.append(("Safety Verification", test_safety_verification()))
    results.append(("Adaptive Mode Selection", test_adaptive_mode_selection()))
    
    # Summary
    print("\n" + "="*70)
    print("TEST SUMMARY")
    print("="*70)
    
    passed = sum(1 for _, result in results if result)
    total = len(results)
    
    for name, result in results:
        status = "✓ PASS" if result else "✗ FAIL"
        print(f"  {status}: {name}")
    
    print(f"\nTotal: {passed}/{total} tests passed")
    
    if passed == total:
        print("\n🎉 ALL TESTS PASSED!")
    else:
        print(f"\n⚠️  {total - passed} test(s) failed")
    
    return passed == total


if __name__ == "__main__":
    success = run_all_tests()
    sys.exit(0 if success else 1)
