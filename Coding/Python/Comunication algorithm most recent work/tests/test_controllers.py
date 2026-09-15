"""Unit tests for controller functionality."""

import numpy as np
import pytest

from config import ControlConfig
from controllers import CLFController, CBFController, HybridCLFCBFController
from core import ChainRobot


def test_clf_controller():
    """Test CLF controller."""
    config = ControlConfig()
    clf = CLFController(config)
    
    robot_pos = np.array([0.0, 0.0])
    goal_pos = np.array([5.0, 5.0])
    
    control = clf.compute_control(robot_pos, goal_pos)
    
    assert control.shape == (2,)
    # Control should point towards goal
    assert control[0] > 0  # Move right
    assert control[1] > 0  # Move up


def test_cbf_safety():
    """Test CBF safety controller."""
    config = ControlConfig()
    cbf = CBFController(config, communication_radius=3.0)
    
    rng = np.random.default_rng(42)
    robot1 = ChainRobot(0, np.array([0.0, 0.0]), rng)
    robot2 = ChainRobot(1, np.array([0.5, 0.0]), rng)  # Very close
    
    control = cbf.compute_safety_control(0, robot1, [robot1, robot2])
    
    assert control.shape == (2,)
    # Control should push away from other robot
    assert control[0] < 0  # Move left (away from robot2)


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
