import numpy as np

class HydrodynamicModel:
    def __init__(self, linear_damping, quad_damping):
        """
        linear_damping, quad_damping: dicts with keys:
            ['Xu', 'Yv', 'Zw', 'Kp', 'Mq', 'Nr']
        """
        self.L = linear_damping
        self.Q = quad_damping

    def compute_damping(self, velocities):
        """
        velocities: [u, v, w, p, q, r]
        Returns: [X, Y, Z, K, M, N] damping forces & moments
        """
        u, v, w, p, q, r = velocities
        damping = np.zeros(6)
        vals = [u, v, w, p, q, r]
        keys = ['Xu', 'Yv', 'Zw', 'Kp', 'Mq', 'Nr']

        for i in range(6):
            lin = self.L.get(keys[i], 0.0)
            quad = self.Q.get(keys[i], 0.0)
            damping[i] = lin * vals[i] + quad * vals[i] * abs(vals[i])
        
        return -damping  # drag always opposes motion
