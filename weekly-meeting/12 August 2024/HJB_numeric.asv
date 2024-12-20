% Parameters
g = 9.81; % gravity (m/s^2)
L = 1.0;  % length of pendulum (m)
m = 1.0;  % mass of pendulum (kg)
dt = 0.01; % time step (s)
T = 10;   % total simulation time (s)

% Cost matrices
Q = diag([1, 0.1]); % State cost
R = 0.1;            % Control cost

% Discretization
theta_range = linspace(-pi/6, pi/6, 50);   % Discretized theta (reduced resolution)
theta_dot_range = linspace(-1, 1, 50); % Discretized theta_dot (reduced resolution)
U_range = linspace(-1, 1, 20);       % Discretized control input (reduced resolution)

% Precompute indices and initialize value function V and policy
V = zeros(length(theta_range), length(theta_dot_range));
policy = zeros(length(theta_range), length(theta_dot_range));

% Precompute grids
[ThetaGrid, ThetaDotGrid] = meshgrid(theta_range, theta_dot_range);

% Value iteration to solve the HJB equation
max_iter = 500;  % Reduced iterations
tolerance = 1e-3;

for iter = 1:max_iter
    V_new = V;
    for i = 1:length(theta_range)
        for j = 1:length(theta_dot_range)
            theta = theta_range(i);
            theta_dot = theta_dot_range(j);
            min_value = Inf;
            for u = U_range
                % Dynamics
                theta_ddot = g/L * sin(theta) + 1/(m*L^2) * u;
                % Next state prediction
                theta_next = theta + theta_dot * dt;
                theta_dot_next = theta_dot + theta_ddot * dt;
                % Wrap theta_next to [-pi, pi]
                theta_next = wrapToPi(theta_next);
                
                % Interpolation to estimate value function
                V_interpolated = interp2(ThetaGrid, ThetaDotGrid, V, theta_next, theta_dot_next, 'linear', Inf);
                
                % Cost-to-go function
                cost_to_go = Q(1,1)*theta^2 + Q(2,2)*theta_dot^2 + R*u^2;
                
                % Update value function
                value = cost_to_go * dt + V_interpolated;
                if value < min_value
                    min_value = value;
                    policy(i,j) = u;
                end
            end
            V_new(i,j) = min_value;
        end
    end
    
    % Check for convergence
    if max(max(abs(V_new - V))) < tolerance
        fprintf('Converged after %d iterations\n', iter);
        break;
    end
    V = V_new;
end

% Simulation
theta0 = 0.2; % Initial angle (rad)
theta_dot0 = 0; % Initial angular velocity (rad/s)
theta = theta0;
theta_dot = theta_dot0;

% Create figure and animated line
figure;
h = animatedline('Color', 'r', 'LineWidth', 2);
axis([-1.5 1.5 -1.5 1.5]);
grid on;

% Run the simulation
for t = 0:dt:T
    % Find the closest state in the grid
    [~, i] = min(abs(theta_range - theta));
    [~, j] = min(abs(theta_dot_range - theta_dot));
    
    % Apply the optimal control
    u = policy(i,j);
    
    % Update dynamics
    theta_ddot = g/L * sin(theta) + 1/(m*L^2) * u;
    theta = theta + theta_dot * dt;
    theta_dot = theta_dot + theta_ddot * dt;
    
    % Update animation
    x = L * sin(theta);
    y = -L * cos(theta);
    clearpoints(h);
    addpoints(h, [0, x], [0, y]);
    drawnow;
    
    pause(dt);
end
