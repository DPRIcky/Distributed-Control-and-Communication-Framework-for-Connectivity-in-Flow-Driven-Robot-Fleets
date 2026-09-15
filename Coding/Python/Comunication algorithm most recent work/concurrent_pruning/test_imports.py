"""Test import scenarios for concurrent_pruning module.

This script verifies that imports work correctly in different contexts:
1. Package import from parent directory
2. Direct module import from within concurrent_pruning directory
3. IDE-style import (simulated)
"""

import sys
from pathlib import Path

def test_package_import():
    """Test importing as a package from parent directory."""
    print("Test 1: Package import from parent directory...")
    try:
        # Ensure parent directory is in path
        parent_dir = Path(__file__).parent.parent
        if str(parent_dir) not in sys.path:
            sys.path.insert(0, str(parent_dir))
        
        from concurrent_pruning import ConcurrentPruningManager
        from concurrent_pruning import LocalLyapunovMonitor, MaxDisagreementMonitor
        print("  ✓ Package import successful!")
        return True
    except ImportError as e:
        print(f"  ✗ Package import failed: {e}")
        return False

def test_direct_import():
    """Test importing directly from module files."""
    print("\nTest 2: Direct module import...")
    try:
        # Add concurrent_pruning to path
        concurrent_dir = Path(__file__).parent
        if str(concurrent_dir) not in sys.path:
            sys.path.insert(0, str(concurrent_dir))
        
        from concurrent_pruning_manager import ConcurrentPruningManager
        from local_lyapunov import LocalLyapunovMonitor
        from max_disagreement import MaxDisagreementMonitor
        print("  ✓ Direct import successful!")
        return True
    except ImportError as e:
        print(f"  ✗ Direct import failed: {e}")
        return False

def test_instantiation():
    """Test that classes can be instantiated."""
    print("\nTest 3: Class instantiation...")
    try:
        from concurrent_pruning import ConcurrentPruningManager
        
        manager = ConcurrentPruningManager(
            num_robots=5,
            mode='lyapunov',
            k_max=50
        )
        print(f"  ✓ Manager created: {manager.num_robots} robots, mode={manager.mode}")
        return True
    except Exception as e:
        print(f"  ✗ Instantiation failed: {e}")
        return False

if __name__ == '__main__':
    print("=" * 70)
    print("CONCURRENT PRUNING IMPORT TESTS")
    print("=" * 70)
    
    results = []
    results.append(("Package import", test_package_import()))
    results.append(("Direct import", test_direct_import()))
    results.append(("Instantiation", test_instantiation()))
    
    print("\n" + "=" * 70)
    print("RESULTS")
    print("=" * 70)
    
    for test_name, passed in results:
        status = "✓ PASS" if passed else "✗ FAIL"
        print(f"{test_name:<30} {status}")
    
    all_passed = all(result[1] for result in results)
    
    print("=" * 70)
    if all_passed:
        print("✓ ALL TESTS PASSED")
    else:
        print("✗ SOME TESTS FAILED")
    print("=" * 70)
    
    sys.exit(0 if all_passed else 1)
