"""Unit tests for core robot functionality."""

import numpy as np
import pytest

from core import ChainRobot, FlowField


def test_robot_initialization():
    """Test robot initialization."""
    rng = np.random.default_rng(42)
    pos = np.array([1.0, 2.0])
    robot = ChainRobot(robot_id=0, position=pos, rng=rng)
    
    assert robot.robot_id == 0
    assert np.allclose(robot.position, pos)
    assert robot.parent is None
    assert len(robot.removed_edges) == 0


def test_robot_step():
    """Test robot position update."""
    rng = np.random.default_rng(42)
    pos = np.array([1.0, 2.0])
    robot = ChainRobot(robot_id=0, position=pos.copy(), rng=rng)
    
    flow = FlowField()
    bounds = np.array([10.0, 10.0])
    
    # Step without control
    old_pos = robot.position.copy()
    robot.step(flow, 0.05, 0.0, bounds)
    
    # Position should change
    assert not np.allclose(robot.position, old_pos)
    # Should stay in bounds
    assert np.all(robot.position >= 0)
    assert np.all(robot.position <= bounds)


def test_flow_field_velocity():
    """Test flow field velocity computation."""
    flow = FlowField()
    pos = np.array([5.0, 5.0])
    vel = flow.velocity(pos, 0.0)
    
    assert vel.shape == (2,)
    assert isinstance(vel, np.ndarray)


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
