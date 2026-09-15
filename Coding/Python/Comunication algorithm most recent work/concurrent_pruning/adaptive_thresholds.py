"""Adaptive threshold scheduling for conservative-to-aggressive pruning.

Mathematical Foundation:
    Thresholds decay exponentially to transition from conservative to aggressive:
    
    For Lyapunov:
        ε(k) = ε_max * exp(-α * k/k_max) + ε_min
    
    For Max Disagreement:
        δ(k) = δ_max * D(k) * exp(-γ * k/k_max) + δ_min * D(k)
        D_threshold(k) = D_0 * (1 - k/k_max)^2 + D_final
    
    Safety margin for λ2:
        μ(k) = μ_max * exp(-β * k/k_max) + μ_min
"""

import numpy as np
from typing import Dict, Optional


class AdaptiveThresholdScheduler:
    """Manage time-varying thresholds for concurrent pruning.
    
    Implements conservative → moderate → aggressive transition by
    exponentially decaying threshold values over consensus iterations.
    
    Attributes:
        k_max: Expected convergence horizon (iterations)
        mode: 'lyapunov', 'max_disagreement', or 'hybrid'
        current_iteration: Current iteration counter
    """
    
    def __init__(
        self,
        k_max: int = 100,
        mode: str = 'lyapunov',
        lyapunov_params: Optional[Dict] = None,
        disagreement_params: Optional[Dict] = None,
        lambda2_params: Optional[Dict] = None
    ):
        """Initialize adaptive threshold scheduler.
        
        Args:
            k_max: Expected convergence horizon
            mode: Pruning mode ('lyapunov', 'max_disagreement', 'hybrid')
            lyapunov_params: Custom parameters for Lyapunov thresholds
            disagreement_params: Custom parameters for disagreement thresholds
            lambda2_params: Custom parameters for λ2 safety margins
        """
        self.k_max = k_max
        self.mode = mode
        self.current_iteration = 0
        
        # Default Lyapunov parameters
        self.lyapunov_params = {
            'epsilon_max': 0.05,     # Initial threshold - very relaxed (requires 5% decrease)
            'epsilon_min': 0.01,     # Final threshold - very relaxed (requires 1% decrease)
            'alpha': 4.0,            # Decay rate - fast transition to aggressive pruning
        }
        if lyapunov_params:
            self.lyapunov_params.update(lyapunov_params)
        
        # Default max disagreement parameters
        self.disagreement_params = {
            'delta_max': 0.2,        # 20% improvement required (rel. to D_l(k))
            'delta_min': 0.01,       # 1% improvement sufficient
            'gamma': 3.0,            # Decay rate for relative threshold
            'abs_threshold_decay': 2.0,  # Exponent for absolute threshold
        }
        if disagreement_params:
            self.disagreement_params.update(disagreement_params)
        
        # Default λ2 safety margin parameters
        self.lambda2_params = {
            'mu_max': 0.15,          # Initial safety margin (moderate)
            'mu_min': 0.03,          # Final safety margin (still safe)
            'beta': 2.0,             # Decay rate
        }
        if lambda2_params:
            self.lambda2_params.update(lambda2_params)
        
        # Store initial values for normalization
        self.initial_lyapunov: Dict[int, float] = {}
        self.initial_disagreement: Dict[int, float] = {}
    
    def set_initial_values(
        self,
        lyapunov_values: Optional[Dict[int, float]] = None,
        disagreement_values: Optional[Dict[int, float]] = None
    ):
        """Store initial metric values for normalization.
        
        Args:
            lyapunov_values: Initial V_l(0) for each robot
            disagreement_values: Initial D_l(0) for each robot
        """
        if lyapunov_values:
            self.initial_lyapunov = lyapunov_values.copy()
        if disagreement_values:
            self.initial_disagreement = disagreement_values.copy()
    
    def get_lyapunov_threshold(
        self,
        robot_id: int,
        current_lyapunov: float
    ) -> float:
        """Compute adaptive Lyapunov threshold ε_l(k).
        
        Formula: ε(k) = max(ε_max * V_l(k) * exp(-α * k/k_max) + ε_min * V_l(k), ε_abs)
        
        Uses CURRENT Lyapunov value V_l(k) instead of initial V_l(0) to adapt
        as consensus converges and Lyapunov decreases. Includes absolute minimum
        threshold to ensure pruning can occur even when V_l is very small.
        
        Args:
            robot_id: Robot ID
            current_lyapunov: Current V_l(k) value
            
        Returns:
            Threshold ε(k)
        """
        k = self.current_iteration
        
        if current_lyapunov == 0:
            return 0.0
        
        eps_max = self.lyapunov_params['epsilon_max']
        eps_min = self.lyapunov_params['epsilon_min']
        alpha = self.lyapunov_params['alpha']
        
        # Exponential decay relative to CURRENT Lyapunov
        epsilon = eps_max * current_lyapunov * np.exp(-alpha * k / self.k_max) + eps_min * current_lyapunov
        
        return epsilon
    
    def get_disagreement_thresholds(
        self,
        robot_id: int,
        current_disagreement: float
    ) -> Dict[str, float]:
        """Compute adaptive disagreement thresholds.
        
        Returns both relative (δ) and absolute (D_threshold) thresholds.
        
        Args:
            robot_id: Robot ID
            current_disagreement: Current D_l(k) value
            
        Returns:
            Dictionary with 'delta' and 'absolute' thresholds
        """
        k = self.current_iteration
        D_0 = self.initial_disagreement.get(robot_id, current_disagreement)
        
        if D_0 == 0:
            return {'delta': 0.0, 'absolute': 0.0}
        
        delta_max = self.disagreement_params['delta_max']
        delta_min = self.disagreement_params['delta_min']
        gamma = self.disagreement_params['gamma']
        decay_exp = self.disagreement_params['abs_threshold_decay']
        
        # Relative threshold: δ(k) = δ_max * D(k) * exp(-γ*k/k_max) + δ_min * D(k)
        delta = (delta_max * current_disagreement * np.exp(-gamma * k / self.k_max) +
                 delta_min * current_disagreement)
        
        # Absolute threshold: D_threshold(k) = D_0 * (1 - k/k_max)^decay_exp + D_final
        progress = min(k / self.k_max, 1.0)
        D_final = 0.01  # Target final disagreement
        absolute = D_0 * (1 - progress) ** decay_exp + D_final
        
        return {
            'delta': delta,
            'absolute': absolute
        }
    
    def get_lambda2_margin(self) -> float:
        """Compute adaptive λ2 safety margin μ(k).
        
        Formula: μ(k) = μ_max * exp(-β * k/k_max) + μ_min
        
        Returns:
            Safety margin μ(k)
        """
        k = self.current_iteration
        mu_max = self.lambda2_params['mu_max']
        mu_min = self.lambda2_params['mu_min']
        beta = self.lambda2_params['beta']
        
        # Exponential decay
        margin = mu_max * np.exp(-beta * k / self.k_max) + mu_min
        
        return margin
    
    def get_phase_name(self) -> str:
        """Get current phase name based on iteration.
        
        Returns:
            Phase name: 'ultraconservative', 'conservative', 'moderate', or 'aggressive'
        """
        progress = self.current_iteration / self.k_max
        
        if progress < 0.2:
            return 'ultraconservative'
        elif progress < 0.5:
            return 'conservative'
        elif progress < 0.8:
            return 'moderate'
        else:
            return 'aggressive'
    
    def get_all_thresholds(
        self,
        robot_id: int,
        current_lyapunov: Optional[float] = None,
        current_disagreement: Optional[float] = None
    ) -> Dict[str, float]:
        """Get all applicable thresholds for current iteration.
        
        Args:
            robot_id: Robot ID
            current_lyapunov: Current V_l(k) if using Lyapunov mode
            current_disagreement: Current D_l(k) if using disagreement mode
            
        Returns:
            Dictionary with all thresholds
        """
        thresholds = {
            'iteration': self.current_iteration,
            'phase': self.get_phase_name(),
            'lambda2_margin': self.get_lambda2_margin()
        }
        
        if self.mode in ['lyapunov', 'hybrid'] and current_lyapunov is not None:
            thresholds['lyapunov_epsilon'] = self.get_lyapunov_threshold(
                robot_id, current_lyapunov
            )
        
        if self.mode in ['max_disagreement', 'hybrid'] and current_disagreement is not None:
            disagreement_thresholds = self.get_disagreement_thresholds(
                robot_id, current_disagreement
            )
            thresholds['disagreement_delta'] = disagreement_thresholds['delta']
            thresholds['disagreement_absolute'] = disagreement_thresholds['absolute']
        
        return thresholds
    
    def increment(self):
        """Increment iteration counter."""
        self.current_iteration += 1
    
    def reset(self):
        """Reset iteration counter."""
        self.current_iteration = 0
        self.initial_lyapunov.clear()
        self.initial_disagreement.clear()
    
    def set_iteration(self, k: int):
        """Manually set iteration counter.
        
        Args:
            k: Iteration number
        """
        self.current_iteration = k
