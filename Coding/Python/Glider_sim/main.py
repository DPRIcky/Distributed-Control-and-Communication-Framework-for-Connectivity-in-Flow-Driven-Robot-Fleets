import numpy as np
import matplotlib.pyplot as plt

from dynamics.rigid_body_dynamics import six_dof_dynamics
from dynamics.forces import compute_restoring_forces
from dynamics.hydrodynamics import HydrodynamicModel
from actuators.buoyancy_control import BuoyancyController
from actuators.mass_shifter import MassShifter

# === AUV Parameters ===
mass = 50.0
volume = 0.0485
fluid_density = 1025
g = 9.81

I_body = [1.0, 1.0, 2.0]
base_cg = [0.0, 0.0, -0.02]
cb = [0.0, 0.0, 0.0]

# === Time Settings ===
dt = 0.01
T = 60.0
steps = int(T / dt)

# === Components ===
buoy = BuoyancyController(base_volume=volume, max_shift_kg=1.0, fluid_density=fluid_density)
shifter = MassShifter(base_cg=base_cg)

drag = HydrodynamicModel(
    linear_damping={'Xu': 5, 'Yv': 8, 'Zw': 10, 'Kp': 0.5, 'Mq': 1.2, 'Nr': 0.8},
    quad_damping={'Xu': 20, 'Yv': 30, 'Zw': 40, 'Kp': 2.0, 'Mq': 3.5, 'Nr': 2.5}
)

# === Initial State ===
state = np.zeros(12)
state[4] = -0.2       # pitch
state[8] = -0.05      # initial heave velocity
state[10] = 0.01      # pitch rate

trajectory = []

# === Simulation Loop ===
for step in range(steps):
    t = step * dt

    # Actuation
    if t < 30:
        buoy.set_command(-1.0)
        shifter.set_command(+1.0)
    else:
        buoy.set_command(+1.0)
        shifter.set_command(-1.0)

    curr_volume = buoy.update(dt)
    curr_cg = shifter.update(dt)
    orientation = state[3:6]

    # Forces
    net_force, net_moment = compute_restoring_forces(
        mass, curr_volume, g, fluid_density,
        center_of_gravity=curr_cg,
        center_of_buoyancy=cb,
        orientation_euler=orientation
    )

    vel = state[6:12]
    Fdamp = drag.compute_damping(vel)
    net_force += Fdamp[:3]
    net_moment += Fdamp[3:]

    dstate = six_dof_dynamics(state, net_force, net_moment, mass, I_body)
    state += dstate * dt
    trajectory.append(np.concatenate(([t], state)))

    # Optional Debug Print
    if step % 500 == 0:
        print(f"t={t:.2f}s | x={state[1]:.3f}, y={state[2]:.3f}, z={state[3]:.3f}, u={state[6]:.3f}, v={state[7]:.3f}, w={state[8]:.3f}")

# === Convert to Array ===
trajectory = np.array(trajectory)

# === Plot 1: Position Components ===
plt.figure(figsize=(10, 5))
plt.plot(trajectory[:, 0], trajectory[:, 1], label='x(t)', color='red')
plt.plot(trajectory[:, 0], trajectory[:, 2], label='y(t)', color='orange')
plt.plot(trajectory[:, 0], trajectory[:, 3], label='z(t)', color='green')
plt.xlabel("Time [s]")
plt.ylabel("Position [m]")
plt.title("AUV Position Over Time")
plt.grid(True)
plt.legend()
plt.show()

# === Plot 2: Velocity Components ===
plt.figure(figsize=(10, 5))
plt.plot(trajectory[:, 0], trajectory[:, 6], label='u(t)', color='blue')
plt.plot(trajectory[:, 0], trajectory[:, 7], label='v(t)', color='cyan')
plt.plot(trajectory[:, 0], trajectory[:, 8], label='w(t)', color='purple')
plt.xlabel("Time [s]")
plt.ylabel("Velocity [m/s]")
plt.title("AUV Linear Velocities Over Time")
plt.grid(True)
plt.legend()
plt.show()
