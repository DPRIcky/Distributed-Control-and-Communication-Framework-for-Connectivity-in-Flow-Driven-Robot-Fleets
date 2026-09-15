"""
Main simulation script for hierarchical multi-agent control.
"""

import numpy as np
import argparse
import time
from typing import List, Dict
import matplotlib.pyplot as plt

from src.core.agent import Agent
from src.core.obstacle import Obstacle
from src.control.cbf_qp_controller import CBFQPController
from src.planning.high_level_planner import HighLevelPlanner
from src.utils.graph_utils import GraphManager, BiddingMechanism
from src.utils.visualization import Visualizer
from scenarios.scenarios import (
    create_apartment_scenario,
    create_crossing_scenario,
    create_narrow_passage_scenario,
    create_simple_scenario
)


class MultiAgentSimulator:
    """Main simulator for multi-agent system."""

    def __init__(self, agents: List[Agent], obstacles: List[Obstacle],
                 dt: float = 0.1, sensing_radius: float = 1.7,
                 safe_distance: float = 0.1, use_high_level_planner: bool = True,
                 use_bidding: bool = True):
        """
        Initialize simulator.

        Args:
            agents: List of agents
            obstacles: List of obstacles
            dt: Time step
            sensing_radius: Sensing radius
            safe_distance: Minimum safe distance
            use_high_level_planner: Enable high-level planner
            use_bidding: Enable bidding mechanism
        """
        self.agents = agents
        self.obstacles = obstacles
        self.dt = dt

        # Controllers and planners
        self.controller = CBFQPController(sensing_radius, safe_distance)
        self.high_level_planner = HighLevelPlanner() if use_high_level_planner else None
        self.use_bidding = use_bidding

        # Network management
        self.graph_manager = GraphManager(sensing_radius)
        self.bidding_mechanism = BiddingMechanism()

        # Simulation data
        self.time = 0.0
        self.trajectory_data = []
        self.computation_times = []

    def step(self):
        """Execute one simulation step."""
        start_time = time.time()

        # Update graph topology
        self.graph_manager.update_graph(self.agents)

        # Update each agent's neighbor list
        for agent in self.agents:
            agent.neighbors = self.graph_manager.get_neighbors(agent.id)

        # High-level planning (deadlock resolution)
        if self.high_level_planner is not None:
            self.high_level_planner.update(self.agents)
            self.high_level_planner.check_goal_swap(self.agents)

        # Bidding mechanism for edge deletion
        if self.use_bidding:
            self._execute_bidding_round()

        # Compute control inputs for each agent
        neighbor_controls = {}

        for agent in self.agents:
            # Get neighbors
            neighbors = [a for a in self.agents if a.id in agent.neighbors]

            # Get obstacles in sensing range
            obstacles_in_range = []
            for obstacle in self.obstacles:
                dist, _ = obstacle.distance_to_point(agent.position)
                if dist < agent.sensing_radius:
                    obstacles_in_range.append(obstacle)

            # Determine connectivity neighbors
            connectivity_neighbors = agent.neighbors.copy()

            # Compute control
            control = self.controller.compute_control(
                agent, neighbors, obstacles_in_range,
                neighbor_controls, connectivity_neighbors
            )

            neighbor_controls[agent.id] = control

        # Update agent states
        for agent in self.agents:
            control = neighbor_controls[agent.id]
            agent.update(control, self.dt)

        # Record data
        self._record_trajectory()

        self.time += self.dt
        comp_time = time.time() - start_time
        self.computation_times.append(comp_time)

    def _execute_bidding_round(self):
        """Execute one round of bidding for edge deletion."""
        # Compute safe neighbors for each agent using connectivity check
        for agent in self.agents:
            agent.safe_neighbors = self.bidding_mechanism.compute_safe_neighbors_with_connectivity(
                agent.id, self.graph_manager
            )

        # Compute bids
        for agent in self.agents:
            if agent.safe_neighbors:
                # Compute bid using distance-based auction
                selected_neighbor, bid_value = self.bidding_mechanism.compute_bid_for_auction(
                    agent, agent.safe_neighbors, self.agents, self.graph_manager
                )

                agent.bid_neighbor = selected_neighbor
                agent.bid_value = bid_value
            else:
                agent.bid_neighbor = None
                agent.bid_value = -1.0

        # Max consensus update
        self.bidding_mechanism.max_consensus_update(self.agents, self.graph_manager)

    def _record_trajectory(self):
        """Record current state for visualization."""
        agent_data = []
        for agent in self.agents:
            agent_data.append((
                agent.position.copy(),
                agent.theta,
                agent.goal.copy(),
                agent.is_leader
            ))

        edges = [(i, j) for i, j in self.graph_manager.graph.edges()]

        self.trajectory_data.append({
            'agents': agent_data,
            'edges': edges,
            'time': self.time
        })

    def run(self, max_time: float = 30.0, goal_threshold: float = 0.15,
           visualize: bool = True, save_interval: int = 10):
        """
        Run simulation.

        Args:
            max_time: Maximum simulation time
            goal_threshold: Distance threshold for goal reaching
            visualize: Enable visualization
            save_interval: Interval for saving frames
        """
        print(f"Starting simulation with {len(self.agents)} agents...")
        print(f"High-level planner: {'Enabled' if self.high_level_planner else 'Disabled'}")
        print(f"Bidding mechanism: {'Enabled' if self.use_bidding else 'Disabled'}")

        # Setup visualization
        visualizer = None
        if visualize:
            visualizer = Visualizer()
            visualizer.setup_plot(self.agents, self.obstacles)
            plt.ion()
            plt.show()

        step = 0
        while self.time < max_time:
            # Execute simulation step
            self.step()

            # Check if all agents reached goals
            all_at_goal = all(agent.is_at_goal(goal_threshold) for agent in self.agents)
            if all_at_goal:
                print(f"\nAll agents reached their goals at t={self.time:.2f}s!")
                break

            # Update visualization
            if visualize and step % save_interval == 0:
                visualizer.update_plot(self.agents, self.graph_manager, show_temp_goals=True)
                plt.pause(0.01)

            # Print progress
            if step % 50 == 0:
                avg_dist = np.mean([agent.distance_to_goal() for agent in self.agents])
                avg_speed = np.mean([agent.get_speed() for agent in self.agents])
                print(f"t={self.time:.2f}s | Avg dist to goal: {avg_dist:.3f} | Avg speed: {avg_speed:.3f}")

            step += 1

        # Final statistics
        print("\n" + "=" * 60)
        print("SIMULATION COMPLETE")
        print("=" * 60)
        print(f"Total time: {self.time:.2f}s")
        print(f"Total steps: {step}")
        print(f"Average computation time per step: {np.mean(self.computation_times):.4f}s")

        final_distances = [agent.distance_to_goal() for agent in self.agents]
        print(f"\nFinal distances to goals:")
        for i, dist in enumerate(final_distances):
            status = "REACHED" if dist < goal_threshold else "NOT REACHED"
            print(f"  Agent {i}: {dist:.3f} [{status}]")

        success_rate = sum(1 for d in final_distances if d < goal_threshold) / len(self.agents) * 100
        print(f"\nSuccess rate: {success_rate:.1f}%")

        if visualize:
            visualizer.update_plot(self.agents, self.graph_manager)
            plt.ioff()
            plt.show()

        return success_rate

    def get_trajectory_data(self):
        """Get recorded trajectory data."""
        return self.trajectory_data


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(description='Multi-Agent Deadlock Resolution Simulation')
    parser.add_argument('--scenario', type=str, default='apartment',
                       choices=['simple', 'apartment', 'crossing', 'narrow'],
                       help='Scenario to simulate')
    parser.add_argument('--max_time', type=float, default=30.0,
                       help='Maximum simulation time (seconds)')
    parser.add_argument('--no_planner', action='store_true',
                       help='Disable high-level planner (for comparison)')
    parser.add_argument('--no_bidding', action='store_true',
                       help='Disable bidding mechanism')
    parser.add_argument('--no_viz', action='store_true',
                       help='Disable visualization')
    parser.add_argument('--dt', type=float, default=0.1,
                       help='Time step (seconds)')

    args = parser.parse_args()

    # Create scenario
    print(f"Loading scenario: {args.scenario}")
    if args.scenario == 'simple':
        agents, obstacles = create_simple_scenario()
    elif args.scenario == 'apartment':
        agents, obstacles = create_apartment_scenario()
    elif args.scenario == 'crossing':
        agents, obstacles = create_crossing_scenario()
    elif args.scenario == 'narrow':
        agents, obstacles = create_narrow_passage_scenario()
    else:
        raise ValueError(f"Unknown scenario: {args.scenario}")

    print(f"Created {len(agents)} agents and {len(obstacles)} obstacles")

    # Create simulator
    simulator = MultiAgentSimulator(
        agents, obstacles,
        dt=args.dt,
        use_high_level_planner=not args.no_planner,
        use_bidding=not args.no_bidding
    )

    # Run simulation
    success_rate = simulator.run(
        max_time=args.max_time,
        visualize=not args.no_viz
    )

    return success_rate


if __name__ == '__main__':
    success_rate = main()
