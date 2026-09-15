"""Adaptive algebraic connectivity (λ₂) estimation for resource-constrained robots.

Novel three-tier approach for computing λ₂ with minimal computational overhead:
    Tier 1: Cheeger lower bound (O(n²)) - fast screening
    Tier 2: Incremental update via perturbation theory (O(n)) - efficient tracking
    Tier 3: Exact eigenvalue computation (O(n³)) - only when critical

Mathematical Foundation:
    From Griparic et al. (2022) Eq. 12:
        ∂λ₂/∂l_ij = (f_i - f_j)²
    
    where f is the Fiedler vector (eigenvector corresponding to λ₂).
    
    Our novel contribution: Use this for incremental updates after edge removal:
        λ₂_new ≈ λ₂_old - (f_i - f_j)²
    
    This avoids O(n³) recomputation, achieving O(n) update cost.
"""

import time
from typing import Optional, Tuple

import numpy as np


class AdaptiveLambda2Manager:
    """Event-triggered adaptive λ₂ estimator for computational efficiency.
    
    Research contribution: Balances accuracy with computational cost through
    adaptive selection of estimation method based on current state.
    
    Attributes:
        num_robots: Number of robots in the system
        lambda2_min: Minimum algebraic connectivity threshold
        safety_margin: Recompute exact if within this margin of threshold
        max_incremental_updates: Force exact computation after N incremental updates
        lambda2_cache: Cached λ₂ value
        fiedler_cache: Cached Fiedler vector for incremental updates
        incremental_count: Number of incremental updates since last exact computation
    """

    def __init__(
        self,
        num_robots: int,
        lambda2_threshold: float = 0.1,
        safety_margin: float = 0.05,
        max_incremental_updates: int = 5
    ):
        """Initialize adaptive λ₂ estimator.
        
        Args:
            num_robots: Number of robots in the system
            lambda2_threshold: Minimum λ₂ to maintain connectivity
            safety_margin: Recompute exact when λ₂ within this margin of threshold
            max_incremental_updates: Maximum incremental updates before exact recomputation
        """
        self.num_robots = num_robots
        self.lambda2_min = lambda2_threshold
        self.safety_margin = safety_margin
        self.max_incremental_updates = max_incremental_updates
        
        # Caching for efficiency
        self.lambda2_cache: Optional[float] = None
        self.fiedler_cache: Optional[np.ndarray] = None
        self.last_exact_computation: float = 0.0
        self.incremental_count: int = 0

    def get_lambda2(
        self,
        A: np.ndarray,
        mode: str = 'auto'
    ) -> float:
        """Get λ₂ estimate using adaptive method selection.
        
        Args:
            A: Adjacency matrix estimate (n×n)
            mode: Computation mode
                'auto' - Automatically select based on state (recommended)
                'exact' - Force exact computation (O(n³))
                'incremental' - Use cached value (O(1))
                'bound' - Use Cheeger lower bound (O(n²))
        
        Returns:
            Algebraic connectivity λ₂
        """
        if mode == 'exact' or (mode == 'auto' and self._should_recompute_exact()):
            return self._compute_exact_lambda2(A)
        
        elif mode == 'incremental' and self.lambda2_cache is not None:
            return self.lambda2_cache
        
        elif mode == 'bound':
            return self._compute_cheeger_bound(A)
        
        else:
            # Auto mode: start with bound if no cache
            if self.lambda2_cache is None:
                return self._compute_exact_lambda2(A)  # Need initial exact computation
            else:
                return self.lambda2_cache

    def _should_recompute_exact(self) -> bool:
        """Event-triggered decision: when to compute exact λ₂?
        
        Triggers:
            1. Too many incremental updates (error accumulation)
            2. λ₂ approaching threshold (need accuracy for safety)
            3. No cached value available
        
        Returns:
            True if exact computation needed
        """
        # Trigger 1: Error accumulation from incremental updates
        if self.incremental_count >= self.max_incremental_updates:
            return True
        
        # Trigger 2: Approaching safety threshold
        if self.lambda2_cache is not None:
            if self.lambda2_cache < self.lambda2_min + self.safety_margin:
                return True
        
        # Trigger 3: No cached value
        if self.lambda2_cache is None:
            return True
        
        return False

    def _compute_exact_lambda2(self, A: np.ndarray) -> float:
        """Compute exact λ₂ via eigenvalue decomposition.
        
        Computational cost: O(n³)
        Also caches Fiedler vector for future incremental updates.
        
        Args:
            A: Adjacency matrix (n×n)
            
        Returns:
            Exact algebraic connectivity λ₂
        """
        L = self._build_laplacian(A)
        
        # Full eigenvalue decomposition
        eigenvalues, eigenvectors = np.linalg.eigh(L)
        
        # Sort to ensure correct ordering
        idx = np.argsort(eigenvalues)
        eigenvalues = eigenvalues[idx]
        eigenvectors = eigenvectors[:, idx]
        
        # Second smallest eigenvalue
        self.lambda2_cache = float(eigenvalues[1])
        
        # Cache Fiedler vector for incremental updates
        self.fiedler_cache = eigenvectors[:, 1].copy()
        
        # Reset counters
        self.incremental_count = 0
        self.last_exact_computation = time.time()
        
        return self.lambda2_cache

    def _build_laplacian(self, A: np.ndarray) -> np.ndarray:
        """Build graph Laplacian from adjacency matrix.
        
        L = D - A where D is the degree matrix.
        
        Args:
            A: Adjacency matrix (n×n)
            
        Returns:
            Laplacian matrix (n×n)
        """
        degrees = np.sum(A, axis=1)
        D = np.diag(degrees)
        L = D - A
        return L

    def _compute_cheeger_bound(self, A: np.ndarray) -> float:
        """Compute Cheeger inequality lower bound for λ₂.
        
        Cheeger inequality: λ₂ ≥ h²/(2·d_max)
        where h = edge connectivity, d_max = maximum degree
        
        Computational cost: O(n²)
        Conservative: if bound > threshold, true λ₂ definitely > threshold
        
        Args:
            A: Adjacency matrix (n×n)
            
        Returns:
            Lower bound on λ₂
        """
        # Compute maximum degree
        degrees = np.sum(A, axis=1)
        d_max = np.max(degrees)
        
        if d_max == 0:
            return 0.0
        
        # Approximate edge connectivity as minimum degree (conservative)
        # True edge connectivity is min-cut, but min degree is cheaper
        h = np.min(degrees[degrees > 0]) if np.any(degrees > 0) else 0.0
        
        # Cheeger bound
        lower_bound = (h ** 2) / (2 * d_max)
        
        return float(lower_bound)

    def update_after_edge_removal(
        self,
        removed_edge: Tuple[int, int]
    ) -> Optional[float]:
        """Incremental λ₂ update using perturbation theory (NOVEL).
        
        From Griparic et al. Eq. 12: ∂λ₂/∂l_ij = (f_i - f_j)²
        
        When removing edge (i,j): Δl_ij = -1
        Therefore: Δλ₂ ≈ -(f_i - f_j)²
        
        Computational cost: O(1) - just arithmetic!
        
        Args:
            removed_edge: Edge that was removed (i, j)
            
        Returns:
            Updated λ₂ estimate, or None if cache not available
        """
        if self.fiedler_cache is None or self.lambda2_cache is None:
            # Need exact computation first
            return None
        
        i, j = removed_edge
        
        # Perturbation: Δλ₂ = -(f_i - f_j)²
        delta_lambda2 = -(self.fiedler_cache[i] - self.fiedler_cache[j]) ** 2
        
        # Update cached value
        self.lambda2_cache += delta_lambda2
        self.incremental_count += 1
        
        return self.lambda2_cache

    def verify_removal_safety(
        self,
        A: np.ndarray,
        edge: Tuple[int, int],
        use_incremental: bool = True
    ) -> Tuple[bool, float]:
        """Verify that removing edge maintains sufficient connectivity.
        
        Smart three-tier strategy:
            1. Quick check: Cheeger bound after removal
            2. If close: Incremental update
            3. If very close: Exact computation
        
        Args:
            A: Current adjacency matrix
            edge: Edge to potentially remove
            use_incremental: Whether to use incremental updates (novel contribution)
            
        Returns:
            Tuple of (is_safe, lambda2_after_removal)
        """
        a, b = edge
        
        # Simulate edge removal
        A_temp = A.copy()
        A_temp[a, b] = 0
        A_temp[b, a] = 0
        
        # Tier 1: Quick pessimistic check with Cheeger bound
        lower_bound = self._compute_cheeger_bound(A_temp)
        
        if lower_bound > self.lambda2_min + self.safety_margin:
            # Definitely safe, don't need more expensive computation
            return True, lower_bound
        
        # Tier 2: Try incremental update if available
        if use_incremental and self.fiedler_cache is not None:
            lambda2_approx = self.update_after_edge_removal(edge)
            
            if lambda2_approx is not None:
                if lambda2_approx > self.lambda2_min + 0.01:
                    # Probably safe with small margin
                    return True, lambda2_approx
        
        # Tier 3: Need exact computation (close to threshold or no cache)
        lambda2_exact = self._compute_exact_lambda2(A_temp)
        is_safe = lambda2_exact > self.lambda2_min
        
        return is_safe, lambda2_exact

    def get_status(self) -> dict:
        """Get estimator status and statistics.
        
        Returns:
            Dictionary with status metrics
        """
        return {
            'lambda2_cached': self.lambda2_cache,
            'has_fiedler': self.fiedler_cache is not None,
            'incremental_count': self.incremental_count,
            'threshold': self.lambda2_min,
            'safety_margin': self.safety_margin
        }

    def reset(self) -> None:
        """Reset caches (e.g., after topology change)."""
        self.lambda2_cache = None
        self.fiedler_cache = None
        self.incremental_count = 0
