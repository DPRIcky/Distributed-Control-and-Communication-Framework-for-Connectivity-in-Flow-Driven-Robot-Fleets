class BuoyancyController:
    def __init__(self, base_volume, max_shift_kg, fluid_density, max_rate=0.0005):
        """
        base_volume: neutral volume (m³)
        max_shift_kg: max amount of water to displace (kg)
        max_rate: max volume change per second (m³/s)
        """
        self.rho = fluid_density
        self.base_volume = base_volume
        self.volume = base_volume  # current displaced volume
        self.max_shift = max_shift_kg / fluid_density  # convert to m³
        self.max_rate = max_rate  # m³/s
        self.command = 0.0  # range [-1, 1], -1 = take in max water, +1 = pump out

    def set_command(self, cmd):
        """Command should be in [-1, 1]"""
        self.command = max(-1.0, min(1.0, cmd))

    def update(self, dt):
        """Update volume based on command and timestep dt"""
        target_volume = self.base_volume + self.command * self.max_shift
        delta = target_volume - self.volume
        max_delta = self.max_rate * dt
        actual_delta = max(-max_delta, min(max_delta, delta))
        self.volume += actual_delta
        return self.volume  # new volume to pass to glider
