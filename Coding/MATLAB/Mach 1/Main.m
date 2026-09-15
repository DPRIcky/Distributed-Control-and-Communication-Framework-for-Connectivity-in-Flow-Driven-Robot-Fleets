clear; clc;
global xg;

% === Environment and Control Parameters ===
x = [0; 0];                % Initial position of the robot
xg = [4; 3.8];             % Target goal location
xo1 = [2; 2.5];            % Center of obstacle 1
xo2 = [4; 4];              % Center of obstacle 2
sigma = 0.7;               % Spread of Gaussian-shaped obstacles
gamma = 15;                % Base CBF gain (safety enforcement)
c = 2;                     % CLF gain (goal convergence)
dt = 0.05;                 % Discrete simulation time step
N = 1000;                  % Maximum allowed simulation steps

% === Data Logging Initialization ===
trajectory = x;            % Initialize trajectory log with start point
velocities = [];           % Speed over time (for velocity diagnostics)
dist_to_obstacles = [];    % Distance to closest obstacle per step
curvatures = [];           % Turning rate (for curvature diagnostics)
times = [];                % Simulation timestamps
last_dir = [];             % To compute angular change in direction

% === Gaussian Obstacle Field Setup ===
[Xgrid, Ygrid] = meshgrid(0:0.1:6, 0:0.1:6);  % Grid for contour plotting
Z1 = exp(-((Xgrid - xo1(1)).^2 + (Ygrid - xo1(2)).^2) / (2 * sigma^2));  % Obstacle 1 field
Z2 = exp(-((Xgrid - xo2(1)).^2 + (Ygrid - xo2(2)).^2) / (2 * sigma^2));  % Obstacle 2 field
Ztotal = max(Z1, Z2);  % Combined Gaussian field (union of obstacle effects)

% === Main Simulation Loop ===
for k = 1:N
    dist_to_goal = norm(x - xg);
    if dist_to_goal < 0.1  % Goal reached threshold
        break;
    end

    % --- Compute Lyapunov and Barrier Constraints ---
    [V, LfV] = compute_clf(x, xg);         % CLF for goal convergence
    [h1, Lfh1] = cbf(x, xo1);              % CBF for obstacle 1
    [h2, Lfh2] = cbf(x, xo2);              % CBF for obstacle 2

    % --- Nominal Goal-Seeking Controller ---
    u_ref = -1.0 * (x - xg);               % Basic gradient descent toward goal

    % --- Add alignment penalty to discourage lateral movement near goal ---
    goal_dir = (xg - x) / norm(xg - x);    % Unit vector toward goal
    penalty_strength = 0.3;                % Scalar penalty factor
    f = -2 * (u_ref + penalty_strength * goal_dir);  % Modified objective

    % --- Scale barrier enforcement depending on distance to goal ---
    if dist_to_goal < 0.3
        beta = 0;                          % Relax barrier near goal
    else
        beta = min(1, 1 / (dist_to_goal + 0.1));  % Adaptive scaling
    end

    % --- Dynamic CBF gain: reduce stiffness when near goal ---
    gamma_adj = gamma * (V > 0.2) + 0.1 * gamma * (V <= 0.2);  % Soft transition

    % --- Quadratic Program Setup ---
    H = 2 * eye(2);                        % Identity matrix for cost
    A = [LfV; -Lfh1; -Lfh2];               % Constraint matrix
    b = [-max(c, 10) * V;                  % CLF constraint
         beta * gamma_adj * h1;            % CBF for obstacle 1
         beta * gamma_adj * h2];           % CBF for obstacle 2

    % --- Solve QP for optimal control input u ---
    u = quadprog(H, f, A, b, [], [], [], [], [], optimoptions('quadprog','Display','off'));

    % --- Adaptive Speed Limiting Near Obstacles ---
    proximity = min(norm(x - xo1), norm(x - xo2));  % Distance to closest obstacle
    r_strict = 0.1;                                 % Red zone (hard constraint)
    r_outer = 1.5;                                  % Soft influence boundary

    decay_scale = (proximity - r_strict) / (r_outer - r_strict);  % Normalized decay [0,1]
    decay_scale = min(max(decay_scale, 0), 1);                    % Clamp for stability

    % --- Exponential velocity decay to smoothly reduce speed near obstacles ---
    max_speed = 1.5 * (1 - exp(-4 * decay_scale)) + 0.05;

    % --- Normalize control input to respect speed limit ---
    u = u * min(1, max_speed / (norm(u) + 1e-5));

    % --- Handle Stuck/Turning-In-Place Situations ---
    direction_to_goal = (xg - x) / (norm(xg - x) + 1e-5);
    cos_angle = dot(direction_to_goal, u) / (norm(u) + 1e-5);
    if norm(u) < 0.05 || cos_angle < 0.5
        u = 0.1 * direction_to_goal;  % Force small step toward goal
    end

    % === Logging Values for Analysis ===
    velocities(end+1) = norm(u);                  % Store current velocity
    times(end+1) = k * dt;                        % Store timestamp
    dist_to_obstacles(end+1) = proximity;         % Store proximity
    if ~isempty(last_dir)
        angle_diff = acos(dot(last_dir, u) / (norm(last_dir)*norm(u)+1e-5));
        curvatures(end+1) = angle_diff / dt;      % Angular change per second
    else
        curvatures(end+1) = 0;                    % No curvature at first step
    end
    last_dir = u;

    % === State Update ===
    x = x + dt * u;               % Euler integration
    trajectory = [trajectory x];  % Store position

    % === Visualization of Path and Environment ===
    figure(1); clf;
    contourf(Xgrid, Ygrid, Ztotal, 20, 'LineColor', 'none'); hold on;
    plot(trajectory(1,:), trajectory(2,:), 'b', 'LineWidth', 2);         % Trajectory
    scatter(xg(1), xg(2), 100, 'g', 'filled');                           % Goal
    scatter(xo1(1), xo1(2), 100, 'r', 'filled');                         % Obstacle 1
    scatter(xo2(1), xo2(2), 100, 'r', 'filled');                         % Obstacle 2
    scatter(trajectory(1,1), trajectory(2,1), 100, 'm', 'filled');       % Start
    title(sprintf('Timestep %d', k)); axis equal; drawnow;

    % === Diagnostics Plots ===
    figure(2); clf;

    % --- Velocity plot ---
    subplot(3,1,1);
    plot(times, velocities, 'k', 'LineWidth', 2);
    ylabel('Velocity (m/s)');
    title('Velocity vs Time'); grid on;

    % --- Obstacle proximity plot ---
    subplot(3,1,2);
    plot(times, dist_to_obstacles, 'r', 'LineWidth', 2);
    ylabel('Dist to Obstacle');
    title('Obstacle Proximity'); grid on;

    % --- Turning rate (curvature) plot ---
    subplot(3,1,3);
    plot(times, curvatures, 'b', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Turning Rate');
    title('Curvature'); grid on;

    drawnow;
end
