function [h, Lfh] = cbf(x, xo)
    % Control Barrier Function (CBF) for obstacle avoidance
    % Inputs:
    %   x  - Current robot position [2x1]
    %   xo - Obstacle center position [2x1]
    % Outputs:
    %   h   - Barrier function value (higher = safer)
    %   Lfh - Lie derivative of the barrier along robot dynamics (∂h/∂x * f(x))

    global xg;  % Access global goal position to shape directional behavior

    % --- Compute Distance to Obstacle ---
    d = norm(x - xo);      % Euclidean distance to obstacle
    r_strict = 0.1;        % Inner (strict) unsafe radius
    d_safe = 0.6;          % Safety enforcement threshold

    % === 1. Hard Constraint Check ===
    if d <= r_strict
        % Robot has entered forbidden zone (within red region)
        h = -Inf;                             % Barrier violated
        grad_h = (x - xo) / d;                % Gradient direction away from center
        Lfh = grad_h' * 1e6;                  % Impose strong penalty
        return;
    end

    % === 2. Barrier Function Definition (log-barrier) ===
    epsilon = 1e-4;                           % Avoid division by zero
    dist_margin = max(d - r_strict, epsilon);% Margin from strict zone
    h = log(dist_margin);                    % Log-barrier: sharply repulsive near red zone
    grad_h = (x - xo) / d;                   % Normalized gradient

    % === 3. Direction Shaping using Tangent Guidance ===
    to_goal = xg - x;                        % Vector to goal
    % Determine "turning direction" to slide along obstacle
    side = sign(dot(to_goal, [0 -1; 1 0] * grad_h)); 
    tangent = side * [0 -1; 1 0] * grad_h;   % Tangent vector (left/right of gradient)

    % If close to goal, avoid adding tangent (stabilize behavior)
    if norm(x - xg) < 0.6
        mixed_grad = grad_h;                 % Pure repulsion
    else
        goal_dir = to_goal / norm(to_goal);  % Normalize direction to goal
        mix_factor = 1 - abs(dot(goal_dir, tangent)); % Misalignment-based weighting
        mixed_grad = grad_h + mix_factor * tangent;   % Blend gradient + tangent
    end

    % === 4. Dynamic Gain Scaling (based on distance) ===
    % Reduce repulsion when far from obstacle
    if d > d_safe + 0.2
        scale = 0.2;                         % Weak influence far away
    else
        % Stronger repulsion as robot nears unsafe radius
        scale = 1.0 + 1.5 * (1 - (d - r_strict) / (d_safe - r_strict));
    end

    % === 5. Compute Lie Derivative ===
    Lfh = mixed_grad' / dist_margin * scale; % Final scaled derivative

end
