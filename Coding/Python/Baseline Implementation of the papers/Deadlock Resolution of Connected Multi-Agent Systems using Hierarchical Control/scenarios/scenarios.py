"""
Test scenarios for multi-agent system.
"""

import numpy as np
from typing import List, Tuple
from src.core.agent import Agent
from src.core.obstacle import Obstacle, create_rectangle_obstacle, create_wall_obstacle


def create_apartment_scenario(sensing_radius: float = 1.7,
                              safe_distance: float = 0.1) -> Tuple[List[Agent], List[Obstacle]]:
    """
    Create apartment scenario with 5 agents.

    Args:
        sensing_radius: Sensing radius for agents
        safe_distance: Minimum safe distance

    Returns:
        (agents, obstacles)
    """
    # Initial positions and goals
    init_positions = np.array([
        [-2.0, -2.0, 0.0],
        [-2.0, -1.5, 0.0],
        [-2.0, -1.0, 0.0],
        [-2.0, -0.5, 0.0],
        [-2.0, 0.0, 0.0],
    ])

    goals = np.array([
        [2.0, 2.0],
        [2.0, 1.5],
        [2.0, 1.0],
        [2.0, 0.5],
        [2.0, 0.0],
    ])

    # Create agents
    agents = []
    for i in range(len(init_positions)):
        agent = Agent(i, init_positions[i], goals[i], sensing_radius, safe_distance)
        agents.append(agent)

    # Create obstacles (apartment layout)
    obstacles = []

    # Outer walls
    obstacles.append(create_wall_obstacle(np.array([-2.5, -2.5]), np.array([2.5, -2.5]), thickness=0.1))  # Bottom
    obstacles.append(create_wall_obstacle(np.array([2.5, -2.5]), np.array([2.5, 2.5]), thickness=0.1))     # Right
    obstacles.append(create_wall_obstacle(np.array([2.5, 2.5]), np.array([-2.5, 2.5]), thickness=0.1))     # Top
    obstacles.append(create_wall_obstacle(np.array([-2.5, 2.5]), np.array([-2.5, -2.5]), thickness=0.1))   # Left

    # Interior obstacles
    obstacles.append(create_rectangle_obstacle(np.array([0.0, 0.0]), 1.0, 1.0))  # Center obstacle
    obstacles.append(create_rectangle_obstacle(np.array([-1.0, 1.0]), 0.6, 0.6))  # Top-left room

    return agents, obstacles


def create_crossing_scenario(sensing_radius: float = 1.7,
                             safe_distance: float = 0.1) -> Tuple[List[Agent], List[Obstacle]]:
    """
    Create crossing scenario with 10 agents.

    Args:
        sensing_radius: Sensing radius for agents
        safe_distance: Minimum safe distance

    Returns:
        (agents, obstacles)
    """
    # Two groups of 5 agents each
    # Group 1: bottom-left to top-right
    # Group 2: bottom-right to top-left

    init_positions = []
    goals = []

    # Group 1: Bottom-left to top-right
    for i in range(5):
        init_positions.append([-2.0, -2.0 + i * 0.4, 0.0])
        goals.append([2.0, 2.0 - i * 0.4])

    # Group 2: Bottom-right to top-left
    for i in range(5):
        init_positions.append([2.0, -2.0 + i * 0.4, np.pi])
        goals.append([-2.0, 2.0 - i * 0.4])

    init_positions = np.array(init_positions)
    goals = np.array(goals)

    # Create agents
    agents = []
    for i in range(len(init_positions)):
        agent = Agent(i, init_positions[i], goals[i], sensing_radius, safe_distance)
        agents.append(agent)

    # Create obstacles
    obstacles = []

    # Boundary walls
    obstacles.append(create_wall_obstacle(np.array([-2.5, -2.5]), np.array([2.5, -2.5]), thickness=0.1))
    obstacles.append(create_wall_obstacle(np.array([2.5, -2.5]), np.array([2.5, 2.5]), thickness=0.1))
    obstacles.append(create_wall_obstacle(np.array([2.5, 2.5]), np.array([-2.5, 2.5]), thickness=0.1))
    obstacles.append(create_wall_obstacle(np.array([-2.5, 2.5]), np.array([-2.5, -2.5]), thickness=0.1))

    # Central obstacle to force interaction
    obstacles.append(create_rectangle_obstacle(np.array([0.0, 0.0]), 0.8, 0.8))

    return agents, obstacles


def create_narrow_passage_scenario(sensing_radius: float = 1.7,
                                   safe_distance: float = 0.1) -> Tuple[List[Agent], List[Obstacle]]:
    """
    Create narrow passage scenario with 24 agents (6 at each corner).

    Args:
        sensing_radius: Sensing radius for agents
        safe_distance: Minimum safe distance

    Returns:
        (agents, obstacles)
    """
    init_positions = []
    goals = []

    # Corner 1: Bottom-left (6 agents) -> Goal: Top-right
    for i in range(2):
        for j in range(3):
            init_positions.append([-2.3 + i * 0.3, -2.3 + j * 0.3, 0.0])
            goals.append([2.3 - i * 0.3, 2.3 - j * 0.3])

    # Corner 2: Top-right (6 agents) -> Goal: Bottom-left
    for i in range(2):
        for j in range(3):
            init_positions.append([2.3 - i * 0.3, 2.3 - j * 0.3, np.pi])
            goals.append([-2.3 + i * 0.3, -2.3 + j * 0.3])

    # Corner 3: Bottom-right (6 agents) -> Goal: Top-left
    for i in range(2):
        for j in range(3):
            init_positions.append([2.3 - i * 0.3, -2.3 + j * 0.3, 0.0])
            goals.append([-2.3 + i * 0.3, 2.3 - j * 0.3])

    # Corner 4: Top-left (6 agents) -> Goal: Bottom-right
    for i in range(2):
        for j in range(3):
            init_positions.append([-2.3 + i * 0.3, 2.3 - j * 0.3, 0.0])
            goals.append([2.3 - i * 0.3, -2.3 + j * 0.3])

    init_positions = np.array(init_positions)
    goals = np.array(goals)

    # Create agents
    agents = []
    for i in range(len(init_positions)):
        agent = Agent(i, init_positions[i], goals[i], sensing_radius, safe_distance)
        agents.append(agent)

    # Create obstacles - four squares creating narrow passages
    obstacles = []

    # Boundary
    obstacles.append(create_wall_obstacle(np.array([-2.5, -2.5]), np.array([2.5, -2.5]), thickness=0.1))
    obstacles.append(create_wall_obstacle(np.array([2.5, -2.5]), np.array([2.5, 2.5]), thickness=0.1))
    obstacles.append(create_wall_obstacle(np.array([2.5, 2.5]), np.array([-2.5, 2.5]), thickness=0.1))
    obstacles.append(create_wall_obstacle(np.array([-2.5, 2.5]), np.array([-2.5, -2.5]), thickness=0.1))

    # Four square obstacles in center
    obstacle_size = 0.7
    gap = 0.3

    obstacles.append(create_rectangle_obstacle(
        np.array([-(obstacle_size + gap) / 2, (obstacle_size + gap) / 2]),
        obstacle_size, obstacle_size
    ))  # Top-left

    obstacles.append(create_rectangle_obstacle(
        np.array([(obstacle_size + gap) / 2, (obstacle_size + gap) / 2]),
        obstacle_size, obstacle_size
    ))  # Top-right

    obstacles.append(create_rectangle_obstacle(
        np.array([-(obstacle_size + gap) / 2, -(obstacle_size + gap) / 2]),
        obstacle_size, obstacle_size
    ))  # Bottom-left

    obstacles.append(create_rectangle_obstacle(
        np.array([(obstacle_size + gap) / 2, -(obstacle_size + gap) / 2]),
        obstacle_size, obstacle_size
    ))  # Bottom-right

    return agents, obstacles


def create_simple_scenario(sensing_radius: float = 1.7,
                           safe_distance: float = 0.1) -> Tuple[List[Agent], List[Obstacle]]:
    """
    Create a simple scenario with 3 agents for testing.

    Args:
        sensing_radius: Sensing radius for agents
        safe_distance: Minimum safe distance

    Returns:
        (agents, obstacles)
    """
    init_positions = np.array([
        [-1.5, 0.0, 0.0],
        [0.0, 0.0, 0.0],
        [1.5, 0.0, 0.0],
    ])

    goals = np.array([
        [1.5, 0.0],
        [0.0, 1.0],
        [-1.5, 0.0],
    ])

    agents = []
    for i in range(len(init_positions)):
        agent = Agent(i, init_positions[i], goals[i], sensing_radius, safe_distance)
        agents.append(agent)

    # Single obstacle in the middle
    obstacles = [create_rectangle_obstacle(np.array([0.0, 0.5]), 0.5, 0.3)]

    return agents, obstacles
