"""
Quick test script to verify installation and basic functionality.
"""

import numpy as np
import sys
import io

# Set UTF-8 encoding for Windows console
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')

def test_imports():
    """Test that all modules can be imported."""
    print("Testing imports...")
    try:
        from src.core import Agent, Obstacle
        from src.control import CBFQPController
        from src.planning import HighLevelPlanner
        from src.utils import GraphManager, BiddingMechanism, Visualizer
        from scenarios import create_simple_scenario
        print("✓ All imports successful")
        return True
    except Exception as e:
        print(f"✗ Import failed: {e}")
        return False


def test_agent_creation():
    """Test agent creation and dynamics."""
    print("\nTesting agent creation...")
    try:
        from src.core import Agent
        agent = Agent(0, np.array([0, 0, 0]), np.array([1, 1]))

        # Test update
        agent.update(np.array([0.1, 0.1]), dt=0.1)

        assert agent.position.shape == (2,)
        assert isinstance(agent.distance_to_goal(), float)
        print("✓ Agent creation and update successful")
        return True
    except Exception as e:
        print(f"✗ Agent test failed: {e}")
        return False


def test_obstacle_creation():
    """Test obstacle creation."""
    print("\nTesting obstacle creation...")
    try:
        from src.core import create_rectangle_obstacle
        obstacle = create_rectangle_obstacle(np.array([0, 0]), 1.0, 1.0)

        dist, closest_point = obstacle.distance_to_point(np.array([2, 0]))
        assert dist > 0
        assert closest_point.shape == (2,)
        print("✓ Obstacle creation successful")
        return True
    except Exception as e:
        print(f"✗ Obstacle test failed: {e}")
        return False


def test_scenario_creation():
    """Test scenario creation."""
    print("\nTesting scenario creation...")
    try:
        from scenarios import create_simple_scenario
        agents, obstacles = create_simple_scenario()

        assert len(agents) == 3
        assert len(obstacles) > 0
        print(f"✓ Scenario created: {len(agents)} agents, {len(obstacles)} obstacles")
        return True
    except Exception as e:
        print(f"✗ Scenario creation failed: {e}")
        return False


def test_graph_manager():
    """Test graph manager."""
    print("\nTesting graph manager...")
    try:
        from src.core import Agent
        from src.utils import GraphManager

        # Create simple agents
        agents = [
            Agent(0, np.array([0, 0, 0]), np.array([1, 0])),
            Agent(1, np.array([1, 0, 0]), np.array([0, 0])),
            Agent(2, np.array([2, 0, 0]), np.array([0, 1])),
        ]

        graph = GraphManager(sensing_radius=1.5)
        graph.update_graph(agents)

        assert graph.is_connected()
        neighbors_0 = graph.get_neighbors(0)
        assert 1 in neighbors_0
        print(f"✓ Graph manager working: {len(graph.graph.nodes())} nodes, {len(graph.graph.edges())} edges")
        return True
    except Exception as e:
        print(f"✗ Graph manager test failed: {e}")
        return False


def test_controller_creation():
    """Test controller creation."""
    print("\nTesting controller creation...")
    try:
        from src.control import CBFQPController
        controller = CBFQPController()
        print("✓ Controller created successfully")
        return True
    except Exception as e:
        print(f"✗ Controller creation failed: {e}")
        return False


def test_planner_creation():
    """Test planner creation."""
    print("\nTesting planner creation...")
    try:
        from src.planning import HighLevelPlanner
        planner = HighLevelPlanner()
        print("✓ Planner created successfully")
        return True
    except Exception as e:
        print(f"✗ Planner creation failed: {e}")
        return False


def main():
    """Run all tests."""
    print("=" * 60)
    print("Running Installation Tests")
    print("=" * 60)

    tests = [
        test_imports,
        test_agent_creation,
        test_obstacle_creation,
        test_scenario_creation,
        test_graph_manager,
        test_controller_creation,
        test_planner_creation,
    ]

    results = []
    for test in tests:
        results.append(test())

    print("\n" + "=" * 60)
    print(f"Test Results: {sum(results)}/{len(results)} passed")
    print("=" * 60)

    if all(results):
        print("\n✓ All tests passed! Installation is successful.")
        print("\nYou can now run the simulation:")
        print("  python main.py --scenario simple")
        return 0
    else:
        print("\n✗ Some tests failed. Please check the error messages above.")
        print("\nMake sure you have installed all dependencies:")
        print("  pip install -r requirements.txt")
        return 1


if __name__ == '__main__':
    sys.exit(main())
