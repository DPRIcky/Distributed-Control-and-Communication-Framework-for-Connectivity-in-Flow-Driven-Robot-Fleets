import numpy as np

def skew(v):
    return np.array([[0, -v[2], v[1]],
                     [v[2], 0, -v[0]],
                     [-v[1], v[0], 0]])

def euler_to_rotation(phi, theta, psi):
    # 3-2-1 rotation: yaw (ψ), pitch (θ), roll (φ)
    cphi, sphi = np.cos(phi), np.sin(phi)
    ctheta, stheta = np.cos(theta), np.sin(theta)
    cpsi, spsi = np.cos(psi), np.sin(psi)

    R = np.array([
        [ctheta * cpsi, sphi * stheta * cpsi - cphi * spsi, cphi * stheta * cpsi + sphi * spsi],
        [ctheta * spsi, sphi * stheta * spsi + cphi * cpsi, cphi * stheta * spsi - sphi * cpsi],
        [-stheta,       sphi * ctheta,                      cphi * ctheta]
    ])
    return R

def six_dof_dynamics(state, forces, moments, mass, I_body):
    """
    Compute the 6DOF time derivative of state based on rigid body dynamics.

    Inputs:
        state: [x, y, z, φ, θ, ψ, u, v, w, p, q, r]
        forces: external force vector [Fx, Fy, Fz]
        moments: external moment vector [Mx, My, Mz]
        mass: scalar
        I_body: [Ixx, Iyy, Izz]
    
    Returns:
        dstate: time derivative of state vector (12,)
    """
    # --- Unpack state ---
    u, v, w = state[6:9]
    p, q, r = state[9:12]
    phi, theta, psi = state[3:6]

    vel_body = np.array([u, v, w])
    omega_body = np.array([p, q, r])

    # --- Kinematics: position in inertial frame ---
    R = euler_to_rotation(phi, theta, psi)
    pos_dot = R @ vel_body  # [dx, dy, dz] in world frame

    # --- Kinematics: Euler angle rates from angular velocity ---
    epsilon = 1e-6  # for numerical safety
    if abs(np.cos(theta)) < epsilon:
        theta = theta + epsilon * np.sign(theta)
    J = np.array([
        [1, np.sin(phi)*np.tan(theta),  np.cos(phi)*np.tan(theta)],
        [0, np.cos(phi),               -np.sin(phi)],
        [0, np.sin(phi)/np.cos(theta),  np.cos(phi)/np.cos(theta)]
    ])
    euler_dot = J @ omega_body

    # --- Dynamics: linear acceleration in body frame ---
    lin_acc = (forces - np.cross(omega_body, mass * vel_body)) / mass

    # --- Dynamics: angular acceleration in body frame ---
    I = np.diag(I_body)
    ang_acc = np.linalg.inv(I) @ (moments - np.cross(omega_body, I @ omega_body))

    # --- Combine into full state derivative ---
    return np.concatenate([pos_dot, euler_dot, lin_acc, ang_acc])
