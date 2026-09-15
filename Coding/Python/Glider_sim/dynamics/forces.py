import numpy as np
from .rigid_body_dynamics import skew, euler_to_rotation

def compute_restoring_forces(
    mass, volume, g,
    fluid_density,
    center_of_gravity,
    center_of_buoyancy,
    orientation_euler
):
    """
    Returns: force [3x1], moment [3x1]
    """
    # Force magnitudes
    W = mass * g                              # weight (N)
    B = fluid_density * volume * g            # buoyancy (N)
    
    # Rotation matrix (body → inertial)
    phi, theta, psi = orientation_euler
    R = euler_to_rotation(phi, theta, psi)

    # Unit vector in z direction (down in NED)
    z_hat = np.array([0, 0, 1])

    # Forces in inertial frame
    Fg_inertial = W * z_hat
    Fb_inertial = -B * z_hat

    # Transform to body frame
    Fg_body = R.T @ Fg_inertial
    Fb_body = R.T @ Fb_inertial

    # Net restoring force
    net_force = Fg_body + Fb_body

    # Moments (torque = r × F)
    M_g = np.cross(center_of_gravity, Fg_body)
    M_b = np.cross(center_of_buoyancy, Fb_body)
    net_moment = M_g + M_b

    return net_force, net_moment
