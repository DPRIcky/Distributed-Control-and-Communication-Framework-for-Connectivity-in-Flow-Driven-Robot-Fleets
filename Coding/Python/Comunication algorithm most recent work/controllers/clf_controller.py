"""Control Lyapunov Function controller."""

import numpy as np

from config.control_config import ControlConfig


class CLFController:
    """Control Lyapunov Function for goal-seeking."""

    def __init__(self, config: ControlConfig):
        """Initialize CLF controller.
        
        Args:
            config: Control configuration
        """
        self.gain = config.clf_gain

    def compute_control(
        self,
        robot_pos: np.ndarray,
        goal_pos: np.ndarray,
        has_parent: bool = True,
        is_root: bool = False
    ) -> np.ndarray:
        """Compute CLF-based control input for goal-seeking.
        
        Args:
            robot_pos: Current robot position
            goal_pos: Goal position
            has_parent: Whether robot is connected to parent in tree
            is_root: Whether robot is the root of tree
            
        Returns:
            Control force vector
        """
        error_to_goal = robot_pos - goal_pos
        distance_to_goal = np.linalg.norm(error_to_goal)

        # Reduce gain for disconnected robots
        if not has_parent and not is_root:
            clf_gain_adjusted = self.gain * 0.15
        else:
            clf_gain_adjusted = self.gain

        if distance_to_goal > 0.1:
            clf_control = -clf_gain_adjusted * error_to_goal
            return clf_control
        
        return np.zeros(2)
