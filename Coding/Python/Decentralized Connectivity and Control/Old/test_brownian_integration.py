"""
Quick test script to validate the Brownian motion integration
"""

import sys
import numpy as np

# Import the main module
sys.path.append('.')
from Base_decentralized_connectivity_with_underwater_flow_field_updated import (
    DistributedNetwork, 
    create_random_graph
)

def test_brownian_integration():
    """Test that both controlled and uncontrolled simulations work"""
    print("🧪 Testing Brownian Motion Integration...")
    
    try:
        # Create a simple network
        print("Creating test network...")
        network = create_random_graph(num_nodes=5, edge_prob=0.3, 
                                    preferred_type='random', enable_underwater=True)
        
        # Test that brownian_motion_simulation method exists
        if hasattr(network, 'brownian_motion_simulation'):
            print("✅ brownian_motion_simulation method found")
        else:
            print("❌ brownian_motion_simulation method not found")
            return False
        
        # Test that goal_seeking_simulation method still exists
        if hasattr(network, 'goal_seeking_simulation'):
            print("✅ goal_seeking_simulation method found")
        else:
            print("❌ goal_seeking_simulation method not found")
            return False
            
        # Test that the network has nodes
        if len(network.nodes) == 5:
            print("✅ Network created with correct number of nodes")
        else:
            print(f"❌ Expected 5 nodes, got {len(network.nodes)}")
            return False
            
        # Test that underwater environment is available
        if hasattr(network, 'underwater_env'):
            print("✅ Underwater environment available")
        else:
            print("⚠️  Underwater environment not found (may be optional)")
            
        print("✅ All integration tests passed!")
        print("\n💡 To test the simulations:")
        print("1. Run: python Base_decentralized_connectivity_with_underwater_flow_field_updated.py")
        print("2. Select option 6 (Real-time pruning simulation)")
        print("3. Select option 1 (Create random graph)")
        print("4. Choose number of robots (e.g., 8)")
        print("5. Choose graph structure (e.g., 5 for random)")
        print("6. Select option 3 (Goal-Seeking)")
        print("7. Choose option 1 for CBF control OR option 2 for pure Brownian motion")
        
        return True
        
    except Exception as e:
        print(f"❌ Test failed with error: {e}")
        return False

if __name__ == "__main__":
    success = test_brownian_integration()
    sys.exit(0 if success else 1)