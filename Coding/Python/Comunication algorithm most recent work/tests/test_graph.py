"""Unit tests for graph utilities."""

import pytest

from graph import EdgeAnalyzer


def test_alternative_path():
    """Test alternative path detection."""
    analyzer = EdgeAnalyzer(num_robots=4)
    
    # Triangle graph: 0-1, 1-2, 0-2
    edges = {(0, 1), (1, 2), (0, 2)}
    
    # Edge (0, 1) has alternative path through (0, 2, 1)
    assert analyzer.has_alternative_path((0, 1), edges)
    
    # But if we only have (0, 1), no alternative
    edges_linear = {(0, 1)}
    assert not analyzer.has_alternative_path((0, 1), edges_linear)


def test_graph_connectivity():
    """Test graph connectivity check."""
    analyzer = EdgeAnalyzer(num_robots=4)
    
    # Fully connected: 0-1, 1-2, 2-3
    edges_connected = {(0, 1), (1, 2), (2, 3)}
    assert analyzer.check_graph_connectivity(edges_connected)
    
    # Disconnected: 0-1, 2-3 (two separate components)
    edges_disconnected = {(0, 1), (2, 3)}
    assert not analyzer.check_graph_connectivity(edges_disconnected)


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
