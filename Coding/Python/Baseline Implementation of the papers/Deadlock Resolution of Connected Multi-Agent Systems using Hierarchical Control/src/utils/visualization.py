"""
Visualization utilities for multi-agent system.
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
from typing import List, Optional
from src.core.agent import Agent
from src.core.obstacle import Obstacle
from src.utils.graph_utils import GraphManager


class Visualizer:
    """Visualize multi-agent system simulation."""

    def __init__(self, workspace_size: tuple = (-3, 3, -3, 3)):
        """
        Initialize visualizer.

        Args:
            workspace_size: (x_min, x_max, y_min, y_max)
        """
        self.workspace_size = workspace_size
        self.fig, self.ax = plt.subplots(figsize=(10, 10))

        # Plot elements
        self.agent_circles = []
        self.agent_arrows = []
        self.agent_texts = []
        self.goal_markers = []
        self.edge_lines = []
        self.obstacle_patches = []

        # Color scheme
        self.agent_color = 'blue'
        self.leader_color = 'red'
        self.goal_color = 'green'
        self.edge_color = 'gray'
        self.obstacle_color = 'black'

    def setup_plot(self, agents: List[Agent], obstacles: List[Obstacle]):
        """
        Setup initial plot.

        Args:
            agents: List of agents
            obstacles: List of obstacles
        """
        self.ax.clear()
        self.ax.set_xlim(self.workspace_size[0], self.workspace_size[1])
        self.ax.set_ylim(self.workspace_size[2], self.workspace_size[3])
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_title('Multi-Agent System with Hierarchical Control')

        # Draw obstacles
        self.obstacle_patches = []
        for obstacle in obstacles:
            poly = patches.Polygon(obstacle.vertices, closed=True,
                                  facecolor=self.obstacle_color, alpha=0.3,
                                  edgecolor=self.obstacle_color, linewidth=2)
            self.ax.add_patch(poly)
            self.obstacle_patches.append(poly)

        # Initialize agent plot elements
        self.agent_circles = []
        self.agent_arrows = []
        self.agent_texts = []
        self.goal_markers = []

        for agent in agents:
            # Agent circle
            circle = plt.Circle(agent.position, 0.1, color=self.agent_color, alpha=0.7)
            self.ax.add_patch(circle)
            self.agent_circles.append(circle)

            # Agent orientation arrow
            dx = 0.15 * np.cos(agent.theta)
            dy = 0.15 * np.sin(agent.theta)
            arrow = self.ax.arrow(agent.position[0], agent.position[1], dx, dy,
                                 head_width=0.08, head_length=0.08, fc=self.agent_color, ec=self.agent_color)
            self.agent_arrows.append(arrow)

            # Agent ID text
            text = self.ax.text(agent.position[0], agent.position[1] + 0.2, f'{agent.id}',
                              ha='center', va='bottom', fontsize=9)
            self.agent_texts.append(text)

            # Goal marker
            goal_marker = self.ax.plot(agent.goal[0], agent.goal[1], 'g*', markersize=15, alpha=0.7)[0]
            self.goal_markers.append(goal_marker)

    def update_plot(self, agents: List[Agent], graph: Optional[GraphManager] = None,
                   show_temp_goals: bool = False):
        """
        Update plot with current agent states.

        Args:
            agents: List of agents
            graph: Graph manager for drawing edges
            show_temp_goals: Show temporary goals
        """
        # Update agent positions and orientations
        for i, agent in enumerate(agents):
            # Update circle position
            self.agent_circles[i].center = agent.position

            # Update color based on leader status
            if agent.is_leader:
                self.agent_circles[i].set_color(self.leader_color)
            else:
                self.agent_circles[i].set_color(self.agent_color)

            # Remove old arrow
            if i < len(self.agent_arrows):
                self.agent_arrows[i].remove()

            # Draw new arrow
            dx = 0.15 * np.cos(agent.theta)
            dy = 0.15 * np.sin(agent.theta)
            arrow = self.ax.arrow(agent.position[0], agent.position[1], dx, dy,
                                 head_width=0.08, head_length=0.08,
                                 fc=self.agent_circles[i].get_facecolor(),
                                 ec=self.agent_circles[i].get_edgecolor())
            self.agent_arrows[i] = arrow

            # Update text position
            self.agent_texts[i].set_position((agent.position[0], agent.position[1] + 0.2))

            # Show temporary goal if enabled
            if show_temp_goals and not np.allclose(agent.goal_temp, agent.goal):
                # Draw line to temporary goal
                self.ax.plot([agent.position[0], agent.goal_temp[0]],
                           [agent.position[1], agent.goal_temp[1]],
                           'r--', alpha=0.5, linewidth=1)

        # Remove old edge lines
        for line in self.edge_lines:
            line.remove()
        self.edge_lines = []

        # Draw edges if graph is provided
        if graph is not None:
            for edge in graph.graph.edges():
                agent_i = next((a for a in agents if a.id == edge[0]), None)
                agent_j = next((a for a in agents if a.id == edge[1]), None)

                if agent_i is not None and agent_j is not None:
                    line = self.ax.plot([agent_i.position[0], agent_j.position[0]],
                                      [agent_i.position[1], agent_j.position[1]],
                                      self.edge_color, alpha=0.3, linewidth=1)[0]
                    self.edge_lines.append(line)

    def save_frame(self, filename: str):
        """Save current frame to file."""
        self.fig.savefig(filename, dpi=150, bbox_inches='tight')

    def show(self):
        """Display the plot."""
        plt.show()

    def close(self):
        """Close the plot."""
        plt.close(self.fig)


def create_animation(trajectory_data: List[dict], obstacles: List[Obstacle],
                    workspace_size: tuple = (-3, 3, -3, 3),
                    save_path: Optional[str] = None) -> FuncAnimation:
    """
    Create animation from trajectory data.

    Args:
        trajectory_data: List of dicts containing agent states at each timestep
        obstacles: List of obstacles
        workspace_size: Workspace boundaries
        save_path: Path to save animation (if provided)

    Returns:
        Animation object
    """
    fig, ax = plt.subplots(figsize=(10, 10))

    def init():
        ax.clear()
        ax.set_xlim(workspace_size[0], workspace_size[1])
        ax.set_ylim(workspace_size[2], workspace_size[3])
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')

        # Draw obstacles
        for obstacle in obstacles:
            poly = patches.Polygon(obstacle.vertices, closed=True,
                                  facecolor='black', alpha=0.3,
                                  edgecolor='black', linewidth=2)
            ax.add_patch(poly)

        return []

    def update(frame):
        ax.clear()
        ax.set_xlim(workspace_size[0], workspace_size[1])
        ax.set_ylim(workspace_size[2], workspace_size[3])
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.set_title(f'Time: {frame * 0.1:.1f}s')

        # Draw obstacles
        for obstacle in obstacles:
            poly = patches.Polygon(obstacle.vertices, closed=True,
                                  facecolor='black', alpha=0.3,
                                  edgecolor='black', linewidth=2)
            ax.add_patch(poly)

        data = trajectory_data[frame]
        agents = data['agents']
        edges = data.get('edges', [])

        # Draw edges
        for edge in edges:
            i, j = edge
            if i < len(agents) and j < len(agents):
                ax.plot([agents[i][0], agents[j][0]],
                       [agents[i][1], agents[j][1]],
                       'gray', alpha=0.3, linewidth=1)

        # Draw agents
        for i, (pos, theta, goal, is_leader) in enumerate(agents):
            color = 'red' if is_leader else 'blue'

            # Agent circle
            circle = plt.Circle(pos, 0.1, color=color, alpha=0.7)
            ax.add_patch(circle)

            # Orientation arrow
            dx = 0.15 * np.cos(theta)
            dy = 0.15 * np.sin(theta)
            ax.arrow(pos[0], pos[1], dx, dy,
                    head_width=0.08, head_length=0.08, fc=color, ec=color)

            # ID
            ax.text(pos[0], pos[1] + 0.2, f'{i}', ha='center', va='bottom', fontsize=9)

            # Goal
            ax.plot(goal[0], goal[1], 'g*', markersize=15, alpha=0.7)

        return []

    anim = FuncAnimation(fig, update, init_func=init, frames=len(trajectory_data),
                        interval=100, blit=False, repeat=True)

    if save_path:
        anim.save(save_path, writer='pillow', fps=10)

    return anim
