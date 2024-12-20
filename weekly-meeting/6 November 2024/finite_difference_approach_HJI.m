% Define parameters for state and time discretization
x_min = -5; x_max = 5;   % State space bounds
dx = 0.1;                % State discretization step
t_min = 0; t_max = 1;    % Time bounds
dt = 0.01;               % Time discretization step
gamma = 0.99;            % Discount factor (close to 1 for near-optimal behavior)

% Define grid for state and time
x_grid = x_min:dx:x_max;
num_states = length(x_grid);

% Initialize value function (high initial values except at the terminal condition)
V = inf(num_states, 1);
V_prev = V; % Placeholder for previous value function

% Parameters for the signed distance function
center = 0;  % Center of the region Ω
radius = 1;  % Radius of the circular boundary

% Define the signed distance function l(x)
l = @(x) signed_distance(x, center, radius);

% Initialize V at terminal condition using signed distance function
for i = 1:num_states
    V(i) = l(x_grid(i)); % Terminal cost
end

% Define control and disturbance sets
possible_controls = -1:0.5:1;      % Example control range
possible_disturbances = -0.5:0.5:0.5; % Example disturbance range

% Define system dynamics function f(x, u, d)
f = @(x, u, d) u + d; % Simple linear dynamics

% Value iteration loop
tolerance = 1e-4; % Convergence tolerance
max_iterations = 1000; % Max iterations for stopping condition
iteration = 0;

while true
    % Backup previous value function
    V_prev = V;
    
    % Loop over each state
    for i = 2:num_states-1
        x = x_grid(i);
        
        % Initialize min-max value as a large number
        min_max_value = inf;
        
        % Loop over controls u (outer maximization)
        for u = possible_controls
            max_u_value = -inf;
            
            % Loop over disturbances d (inner minimization)
            for d = possible_disturbances
                % Calculate next state using dynamics
                x_next = x + dt * f(x, u, d);
                
                % Interpolate to find the value function at the next state
                if x_next < x_min
                    x_next = x_min;
                elseif x_next > x_max
                    x_next = x_max;
                end
                % Use linear interpolation to find V at x_next
                V_next = interp1(x_grid, V_prev, x_next, 'linear', 'extrap');
                
                % Calculate cost-to-go using signed distance function
                cost_to_go = V_next + dt * l(x); % Signed distance penalty
                
                % Update max_u_value as the maximum over control values
                max_u_value = max(max_u_value, cost_to_go);
            end
            
            % Update min_max_value as the minimum over disturbance values
            min_max_value = min(min_max_value, max_u_value);
        end
        
        % Update V for the current state based on min-max value
        V(i) = min(V(i), min_max_value);
    end
    
    % Check for convergence
    if max(abs(V - V_prev)) < tolerance
        disp('Value function converged.');
        break;
    end
    
    % Stop if max iterations reached
    iteration = iteration + 1;
    if iteration >= max_iterations
        disp('Max iterations reached without full convergence.');
        break;
    end
end

% Plot the final value function
figure;
plot(x_grid, V);
xlabel('State (x)');
ylabel('Value Function V(x, 0)');
title('Converged Value Function with Signed Distance Cost');
grid on;

% Signed distance function definition
function d = signed_distance(x, center, radius)
    % Calculate the Euclidean distance to the boundary
    distance_to_boundary = abs(x - center) - radius;
    
    % Apply signed distance convention
    if abs(x - center) <= radius
        d = distance_to_boundary; % Inside the region Ω
    else
        d = -distance_to_boundary; % Outside the region Ω
    end
end
