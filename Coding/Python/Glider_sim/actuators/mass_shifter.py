class MassShifter:
    def __init__(self, base_cg=[0.0, 0.0, -0.02], max_shift=0.05, max_rate=0.005):
        """
        base_cg: default center of gravity [x, y, z] in body frame
        max_shift: max shift distance in ±x (meters)
        max_rate: max shift speed (m/s)
        """
        self.base_cg = base_cg.copy()
        self.max_shift = max_shift
        self.max_rate = max_rate
        self.command = 0.0  # normalized [-1, 1]
        self.x_offset = 0.0  # dynamic x-shift from base_cg

    def set_command(self, cmd):
        """Set target shift direction, cmd in [-1, 1]"""
        self.command = max(-1.0, min(1.0, cmd))

    def update(self, dt):
        """Update x-offset based on command and time step"""
        target_offset = self.command * self.max_shift
        delta = target_offset - self.x_offset
        max_delta = self.max_rate * dt
        actual_delta = max(-max_delta, min(max_delta, delta))
        self.x_offset += actual_delta

        # Return updated center of gravity [x, y, z]
        return [self.base_cg[0] + self.x_offset, self.base_cg[1], self.base_cg[2]]
