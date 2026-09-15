"""
High-level planner for temporary goal assignment (Algorithm 1).
"""

import numpy as np
from typing import List, Dict, Set, Tuple, Optional
from src.core.agent import Agent


class HighLevelPlanner:
    """
    High-level planner for deadlock resolution via leader-follower assignment.
    Implements Algorithm 1 from the paper.
    """

    def __init__(self, u_min: float = 0.01, d_min: float = 0.5):
        """
        Initialize high-level planner.

        Args:
            u_min: Minimum average speed threshold for deadlock detection
            d_min: Minimum distance from goal to assign temporary goal
        """
        self.u_min = u_min
        self.d_min = d_min

        # Leader-follower assignments
        self.leader_id = None
        self.follower_assignments = {}  # follower_id -> leader_id

        # Deadlock detection
        self.in_deadlock_mode = False

    def update(self, agents: List[Agent]) -> bool:
        """
        Update leader-follower assignments if deadlock is detected.

        Args:
            agents: List of all agents

        Returns:
            True if deadlock mode is activated
        """
        # Compute average speed of the MAS
        avg_speed = np.mean([agent.get_speed() for agent in agents])

        # Check if deadlock
        if avg_speed < self.u_min:
            if not self.in_deadlock_mode:
                print(f"Deadlock detected! Average speed: {avg_speed:.4f} < {self.u_min}")
                self.in_deadlock_mode = True
                self._assign_leader_follower(agents)
                return True
        else:
            if self.in_deadlock_mode:
                print(f"Exiting deadlock mode. Average speed: {avg_speed:.4f}")
                self.in_deadlock_mode = False
                self._reset_assignments(agents)

        # If in deadlock mode, check if leader is stuck
        if self.in_deadlock_mode and self.leader_id is not None:
            leader = next((a for a in agents if a.id == self.leader_id), None)
            if leader is not None:
                if leader.get_speed() < self.u_min and leader.distance_to_goal() >= self.d_min:
                    # Assign temporary goal for leader
                    self._assign_temporary_goal_for_leader(leader, agents)

        return self.in_deadlock_mode

    def _assign_leader_follower(self, agents: List[Agent]):
        """
        Assign leader and followers according to Algorithm 1.

        Args:
            agents: List of all agents
        """
        N = len(agents)

        # Step 1: Choose leader
        # lead = arg min_i ||p_i - p_gi|| / ||p_dot_i|_{p_gi,temp}||
        leader_scores = []
        for agent in agents:
            dist_to_goal = agent.distance_to_temp_goal()
            speed = max(agent.get_speed(), 1e-6)  # Avoid division by zero
            score = dist_to_goal / speed
            leader_scores.append((score, agent.id))

        leader_scores.sort()
        self.leader_id = leader_scores[0][1]

        # Set leader flag
        for agent in agents:
            agent.is_leader = (agent.id == self.leader_id)
            agent.leader_id = None

        print(f"Leader assigned: Agent {self.leader_id}")

        # Step 2: Assign followers
        A_lead = {self.leader_id}
        self.follower_assignments = {}

        for k in range(1, N):
            # Find follower_k: agent not in A_lead that is closest to any agent in A_lead
            min_dist = np.inf
            follower_k = None
            leader_k = None

            for agent in agents:
                if agent.id in A_lead:
                    continue

                for lead_id in A_lead:
                    lead_agent = next((a for a in agents if a.id == lead_id), None)
                    if lead_agent is None:
                        continue

                    dist = agent.distance_to_agent(lead_agent)
                    if dist < min_dist:
                        min_dist = dist
                        follower_k = agent.id
                        leader_k = lead_id

            if follower_k is None:
                break

            # Add follower to A_lead
            A_lead.add(follower_k)
            self.follower_assignments[follower_k] = leader_k

            # Assign temporary goal for follower (if far enough from goal)
            follower = next((a for a in agents if a.id == follower_k), None)
            if follower is not None and follower.distance_to_goal() >= self.d_min:
                leader_agent = next((a for a in agents if a.id == leader_k), None)
                if leader_agent is not None:
                    follower.set_temporary_goal(leader_agent.position)
                    follower.leader_id = leader_k
                    print(f"Agent {follower_k} follows Agent {leader_k}")

    def _assign_temporary_goal_for_leader(self, leader: Agent, agents: List[Agent]):
        """
        Assign temporary goal for leader to maximize movement speed.

        Args:
            leader: Leader agent
            agents: All agents
        """
        # Sample points on a circle around the leader
        n_samples = 16
        max_speed = 0.0
        best_goal = leader.goal.copy()

        for i in range(n_samples):
            angle = 2 * np.pi * i / n_samples
            # Sample at unit distance from current position
            sample_goal = leader.position + np.array([np.cos(angle), np.sin(angle)])

            # Estimate speed towards this goal (heuristic)
            direction = sample_goal - leader.position
            direction_norm = np.linalg.norm(direction)
            if direction_norm > 0:
                direction = direction / direction_norm
                # Simple heuristic: speed is higher if no obstacles in that direction
                estimated_speed = 1.0  # Base speed

                # Check if this direction is blocked by nearby agents
                for other in agents:
                    if other.id == leader.id:
                        continue
                    rel_pos = other.position - leader.position
                    projection = np.dot(rel_pos, direction)
                    if projection > 0 and projection < 1.0:
                        # Agent is in this direction
                        estimated_speed *= 0.5

                if estimated_speed > max_speed:
                    max_speed = estimated_speed
                    best_goal = sample_goal

        leader.set_temporary_goal(best_goal)
        print(f"Leader {leader.id} assigned temporary goal at {best_goal}")

    def _reset_assignments(self, agents: List[Agent]):
        """
        Reset all temporary goals and leader-follower assignments.

        Args:
            agents: List of all agents
        """
        self.leader_id = None
        self.follower_assignments = {}

        for agent in agents:
            agent.reset_temporary_goal()
            agent.is_leader = False
            agent.leader_id = None

    def check_goal_swap(self, agents: List[Agent]):
        """
        Check if agents that reached goals should swap with stuck agents.

        Args:
            agents: List of all agents
        """
        # Find agents at their goals
        agents_at_goal = [a for a in agents if a.is_at_goal(threshold=0.15)]

        # Find agents stuck (not at goal and low speed)
        agents_stuck = [a for a in agents if not a.is_at_goal(threshold=0.15) and a.get_speed() < self.u_min]

        # For each stuck agent, check if it's blocked by an agent at goal
        for stuck_agent in agents_stuck:
            for goal_agent in agents_at_goal:
                dist = stuck_agent.distance_to_agent(goal_agent)
                if dist < 2 * stuck_agent.safe_distance:
                    # Swap goals
                    print(f"Swapping goals: Agent {stuck_agent.id} <-> Agent {goal_agent.id}")
                    temp_goal = stuck_agent.goal.copy()
                    stuck_agent.goal = goal_agent.goal.copy()
                    goal_agent.goal = temp_goal
                    stuck_agent.reset_temporary_goal()
                    goal_agent.reset_temporary_goal()
                    break
