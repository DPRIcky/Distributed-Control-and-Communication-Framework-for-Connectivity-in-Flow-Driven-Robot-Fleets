"""Unit tests for concurrent pruning components."""

import numpy as np
import pytest
from typing import Set

from concurrent_pruning.local_lyapunov import LocalLyapunovMonitor
from concurrent_pruning.max_disagreement import MaxDisagreementMonitor
from concurrent_pruning.adaptive_thresholds import AdaptiveThresholdScheduler
from concurrent_pruning.concurrent_pruning_manager import ConcurrentPruningManager
from core.types import Edge


class TestLocalLyapunov:
    """Test Local Lyapunov function computation."""
    
    def test_initialization(self):
        """Test monitor initialization."""
        monitor = LocalLyapunovMonitor(num_robots=5)
        assert monitor.num_robots == 5
        assert len(monitor.lyapunov_history) == 5
    
    def test_compute_lyapunov(self):
        """Test Lyapunov computation."""
        monitor = LocalLyapunovMonitor(num_robots=3)
        
        # Create test estimates
        A_estimates = {
            0: np.array([[0, 0.8, 0.2], [0.8, 0, 0.5], [0.2, 0.5, 0]]),
            1: np.array([[0, 0.7, 0.3], [0.7, 0, 0.6], [0.3, 0.6, 0]]),
            2: np.array([[0, 0.9, 0.1], [0.9, 0, 0.4], [0.1, 0.4, 0]])
        }
        
        neighbors = {0, 1}
        V_l = monitor.compute_local_lyapunov(0, A_estimates, neighbors)
        
        assert V_l >= 0  # Lyapunov must be non-negative
        assert len(monitor.lyapunov_history[0]) == 1
    
    def test_convergence_tracking(self):
        """Test convergence status."""
        monitor = LocalLyapunovMonitor(num_robots=3)
        
        # Simulate decreasing Lyapunov
        for k in range(10):
            monitor.lyapunov_history[0].append(10.0 * np.exp(-0.3 * k))
            monitor.initial_lyapunov[0] = 10.0
        
        status = monitor.get_convergence_status(0, window_size=5)
        
        assert status['iterations'] == 10
        assert status['decrease_rate'] > 0  # Should be decreasing


class TestMaxDisagreement:
    """Test Max Disagreement metric computation."""
    
    def test_initialization(self):
        """Test monitor initialization."""
        monitor = MaxDisagreementMonitor(num_robots=5)
        assert monitor.num_robots == 5
        assert len(monitor.disagreement_history) == 5
    
    def test_compute_disagreement(self):
        """Test disagreement computation."""
        monitor = MaxDisagreementMonitor(num_robots=3)
        
        # Create test estimates with clear worst neighbor
        A_estimates = {
            0: np.array([[0, 0.8, 0.2], [0.8, 0, 0.5], [0.2, 0.5, 0]]),
            1: np.array([[0, 0.8, 0.2], [0.8, 0, 0.5], [0.2, 0.5, 0]]),  # Same as robot 0
            2: np.array([[0, 0.1, 0.9], [0.1, 0, 0.1], [0.9, 0.1, 0]])   # Very different
        }
        
        neighbors = {1, 2}
        D_l, worst = monitor.compute_max_disagreement(0, A_estimates, neighbors)
        
        assert D_l >= 0
        assert worst == 2  # Robot 2 should be worst neighbor
    
    def test_average_disagreement(self):
        """Test average disagreement computation."""
        monitor = MaxDisagreementMonitor(num_robots=3)
        
        A_estimates = {
            0: np.eye(3),
            1: 0.5 * np.eye(3),
            2: 0.2 * np.eye(3)
        }
        
        neighbors = {1, 2}
        avg = monitor.compute_average_disagreement(0, A_estimates, neighbors)
        
        assert avg >= 0
        assert avg < monitor.compute_max_disagreement(0, A_estimates, neighbors)[0]


class TestAdaptiveThresholds:
    """Test adaptive threshold scheduling."""
    
    def test_initialization(self):
        """Test scheduler initialization."""
        scheduler = AdaptiveThresholdScheduler(k_max=100, mode='lyapunov')
        assert scheduler.k_max == 100
        assert scheduler.mode == 'lyapunov'
        assert scheduler.current_iteration == 0
    
    def test_lyapunov_threshold_decay(self):
        """Test Lyapunov threshold decreases over time."""
        scheduler = AdaptiveThresholdScheduler(k_max=100, mode='lyapunov')
        scheduler.set_initial_values(lyapunov_values={0: 10.0})
        
        # Early iteration
        scheduler.set_iteration(0)
        epsilon_early = scheduler.get_lyapunov_threshold(0, 10.0)
        
        # Late iteration
        scheduler.set_iteration(80)
        epsilon_late = scheduler.get_lyapunov_threshold(0, 10.0)
        
        assert epsilon_early > epsilon_late  # Should decay
        assert epsilon_late > 0  # Should never reach zero
    
    def test_phase_transitions(self):
        """Test phase name changes."""
        scheduler = AdaptiveThresholdScheduler(k_max=100)
        
        scheduler.set_iteration(5)
        assert scheduler.get_phase_name() == 'ultraconservative'
        
        scheduler.set_iteration(30)
        assert scheduler.get_phase_name() == 'conservative'
        
        scheduler.set_iteration(60)
        assert scheduler.get_phase_name() == 'moderate'
        
        scheduler.set_iteration(90)
        assert scheduler.get_phase_name() == 'aggressive'
    
    def test_lambda2_margin_decay(self):
        """Test λ2 safety margin decay."""
        scheduler = AdaptiveThresholdScheduler(k_max=100)
        
        scheduler.set_iteration(0)
        margin_early = scheduler.get_lambda2_margin()
        
        scheduler.set_iteration(80)
        margin_late = scheduler.get_lambda2_margin()
        
        assert margin_early > margin_late


class TestConcurrentPruningManager:
    """Test concurrent pruning manager."""
    
    def test_initialization(self):
        """Test manager initialization."""
        manager = ConcurrentPruningManager(
            num_robots=5,
            mode='lyapunov',
            k_max=50
        )
        
        assert manager.num_robots == 5
        assert manager.mode == 'lyapunov'
        assert manager.lyapunov_monitor is not None
        assert manager.current_iteration == 0
    
    def test_mode_selection(self):
        """Test different modes initialize correct monitors."""
        # Lyapunov mode
        manager_lyap = ConcurrentPruningManager(num_robots=5, mode='lyapunov')
        assert manager_lyap.lyapunov_monitor is not None
        assert manager_lyap.disagreement_monitor is None
        
        # Max disagreement mode
        manager_disagree = ConcurrentPruningManager(num_robots=5, mode='max_disagreement')
        assert manager_disagree.lyapunov_monitor is None
        assert manager_disagree.disagreement_monitor is not None
        
        # Hybrid mode
        manager_hybrid = ConcurrentPruningManager(num_robots=5, mode='hybrid')
        assert manager_hybrid.lyapunov_monitor is not None
        assert manager_hybrid.disagreement_monitor is not None
    
    def test_initialization_and_step(self):
        """Test initialization and single step execution."""
        num_robots = 5
        manager = ConcurrentPruningManager(num_robots=num_robots, mode='lyapunov')
        
        # Create simple topology
        positions = np.random.rand(num_robots, 2)
        edges: Set[Edge] = {(0, 1), (1, 2), (2, 3), (3, 4), (0, 2)}  # Has redundant edge
        
        # Initialize
        manager.initialize_robot_knowledge(positions, edges)
        
        # Run one step
        report = manager.concurrent_step(positions, edges, debug=False)
        
        assert 'iteration' in report
        assert 'edges_before' in report
        assert 'phase' in report
        assert manager.current_iteration == 1
    
    def test_summary(self):
        """Test summary generation."""
        manager = ConcurrentPruningManager(num_robots=5, mode='lyapunov')
        manager.current_iteration = 10
        manager.pruned_edges = [(0, 1), (2, 3)]
        
        summary = manager.get_summary()
        
        assert summary['total_iterations'] == 10
        assert summary['total_edges_pruned'] == 2
        assert summary['mode'] == 'lyapunov'


def test_integration_simple_pruning():
    """Integration test: simple topology pruning."""
    num_robots = 4
    
    # Create square with diagonal (redundant edge)
    positions = np.array([
        [0, 0],
        [1, 0],
        [1, 1],
        [0, 1]
    ])
    
    edges: Set[Edge] = {(0, 1), (1, 2), (2, 3), (3, 0), (0, 2)}  # Diagonal 0-2 is redundant
    
    manager = ConcurrentPruningManager(
        num_robots=num_robots,
        mode='lyapunov',
        k_max=50
    )
    
    manager.initialize_robot_knowledge(positions, edges)
    
    # Run for several iterations
    for _ in range(30):
        report = manager.concurrent_step(positions, edges, debug=False)
        
        if report['pruned_edge']:
            print(f"Pruned edge: {report['pruned_edge']}")
        
        # Stop if reached tree
        if len(edges) == num_robots - 1:
            break
    
    summary = manager.get_summary()
    
    # Should have pruned at least one edge
    assert summary['total_edges_pruned'] >= 1
    # Should not over-prune (maintain connectivity)
    assert len(edges) >= num_robots - 1


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
