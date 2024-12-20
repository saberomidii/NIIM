% Dubins Car Dynamic Programming in MATLAB
% This script performs dynamic programming to find the optimal policy for Dubins's car
% It includes state transitions, cost functions, backward iteration, and simulation

% Clear workspace and command window
clear; clc; close all;

%% ------------------------------
% Dynamics of Dubins's Car
% ------------------------------
function next_state = dynamics(state, action, delta_t)
    % Computes the next state given the current state and action
    % state: [x; y; theta]
    % action: [omega; v]
    % delta_t: time step
    x = state(1);
    y = state(2);
    theta = state(3);
    omega = action(1);
    v = action(2);
    
    x_next = x + v * cos(theta) * delta_t;
    y_next = y + v * sin(theta) * delta_t;
    theta_next = mod(theta + omega * delta_t, 2*pi);  % Ensure theta wraps around 2π
    
    next_state = [x_next; y_next; theta_next];
end

%% ------------------------------
% Cost Function Penalizing Proximity to Boundaries
% ------------------------------
function cost = g(state)
    % Computes the cost based on the state's proximity to boundaries
    % state: [x; y; theta]
    x = state(1);
    y = state(2);
    
    % Define bounds
    min_x = -1.0; max_x = 1.0;
    min_y = -1.0; max_y = 1.0;
    
    % Cost function: penalize being close to boundaries
    if (x >= min_x) && (x <= max_x) && (y >= min_y) && (y <= max_y)
        cost = -min([x - min_x, max_x - x, y - min_y, max_y - y]);
    else
        dx = max([min_x - x, 0, x - max_x]);
        dy = max([min_y - y, 0, y - max_y]);
        cost = sqrt(dx^2 + dy^2);
    end
end

%% ------------------------------
% Function to Get the Closest Grid Indices for a Given State
% ------------------------------
function [x_idx, y_idx, theta_idx] = get_indices(state, x_grid, y_grid, theta_grid)
    % Finds the closest indices in the grid for the given state
    x = state(1);
    y = state(2);
    theta = state(3);
    
    [~, x_idx] = min(abs(x_grid - x));
    [~, y_idx] = min(abs(y_grid - y));
    
    % Handle theta wrapping around 2π by finding the minimal difference
    theta_diff = abs(theta_grid - theta);
    theta_diff = min(theta_diff, 2*pi - theta_diff);
    [~, theta_idx] = min(theta_diff);
end

%% ------------------------------
% Function to Plot Heatmap of V for Given Time Step k and Theta Index ti
% ------------------------------
function plot_V_heatmap(V_slice, x_grid, y_grid, k, ti)
    % Plots the heatmap for a slice of the value function
    % V_slice: 2D slice of V at time step k and theta index ti
    % x_grid, y_grid: grid vectors
    % k: time step
    % ti: theta index
    
    figure;
    imagesc(x_grid, y_grid, V_slice);
    set(gca, 'YDir', 'normal');
    colorbar;
    xlabel('x');
    ylabel('y');
    title(sprintf('Value Function V at time step %d, θ = %.2f rad', k, ti));
end

%% ------------------------------
% Defining Grids
% ------------------------------
% Define state bounds and grids
lower_x = -5.0; upper_x = 5.0;
lower_y = -5.0; upper_y = 5.0;
lower_theta = 0.0; upper_theta = 2*pi;
lower_omega = -10.0; upper_omega = 10.0;
lower_v = -2.0; upper_v = 2.0;

num_x = 10;
num_y = 10;
num_theta = 30;
num_omega = 20;
num_v = 20;

x_grid = round(linspace(lower_x, upper_x, num_x), 1);
y_grid = round(linspace(lower_y, upper_y, num_y), 1);
theta_grid = linspace(lower_theta, upper_theta, num_theta);
omega_grid = linspace(lower_omega, upper_omega, num_omega);
v_grid = linspace(lower_v, upper_v, num_v);  % Corrected length

%% ------------------------------
% Initialize Value Function and Policy
% ------------------------------
time_steps = 20;  % Number of time steps
delta_t = 0.1;    % Time step duration

% Initialize V with Inf
V = Inf(time_steps + 1, num_x, num_y, num_theta);

% Initialize policy with [0; 0] actions
policy = zeros(time_steps, num_x, num_y, num_theta, 2);  % Last dimension for [omega, v]

%% ------------------------------
% Initialize Terminal Cost
% ------------------------------
disp('Initializing terminal cost...');
for xi = 1:num_x
    for yi = 1:num_y
        for ti = 1:num_theta
            state = [x_grid(xi); y_grid(yi); theta_grid(ti)];
            V(time_steps + 1, xi, yi, ti) = g(state);
        end
    end
end
disp('Terminal cost initialization completed.');

%% --------------------------------------------------------------------------
% Perform Backward Iteration to Compute Value Function and Policy
% ---------------------------------------------------------------------------
disp('Starting backward iteration to compute Value Function and Policy...');
for k = time_steps:-1:1
    fprintf('Calculating optimal policy for time step %d...\n', k);
    for xi = 1:num_x
        for yi = 1:num_y
            for ti = 1:num_theta
                x = x_grid(xi);
                y = y_grid(yi);
                theta = theta_grid(ti);
                state = [x; y; theta];
                min_value = Inf;
                best_action = [0.0; 0.0];  % Default action
                
                for omega = omega_grid
                    for v = v_grid
                        action = [omega; v];
                        % Compute the next state
                        next_state = dynamics(state, action, delta_t);
                        x_next = next_state(1);
                        y_next = next_state(2);
                        theta_next = next_state(3);
                        
                        % Check if next_state is within bounds
                        if (x_next >= lower_x) && (x_next <= upper_x) && (y_next >= lower_y) && (y_next <= upper_y)
                            % Find indices of next_state
                            [x_next_idx, y_next_idx, theta_next_idx] = get_indices(next_state, x_grid, y_grid, theta_grid);
                            
                            % Ensure indices are within bounds
                            if (x_next_idx >=1 && x_next_idx <= num_x) && ...
                               (y_next_idx >=1 && y_next_idx <= num_y) && ...
                               (theta_next_idx >=1 && theta_next_idx <= num_theta)
                                % Compute future value
                                future_value = V(k + 1, x_next_idx, y_next_idx, theta_next_idx);
                            else
                                future_value = Inf;
                            end
                        else
                            future_value = Inf;
                        end
                        
                        % Compute the cost associated with the next state
                        cost = g(next_state);
                        % total_value = cost + future_value;  % Assuming additive cost
                        
                        total_value = min(cost , future_value);  % Assuming additive cost

                        % Update minimum value and best actions
                        if total_value < min_value
                            min_value = total_value;
                            best_action = action;
                        end
                    end
                end
                
                % Update Value Function and Policy
                V(k, xi, yi, ti) = min_value;
                policy(k, xi, yi, ti, :) = best_action;
            end
        end
    end
end
disp('Backward iteration completed.');

%% ------------------------------
% Plotting Value Function Heatmaps
% ------------------------------
disp('Plotting Value Function heatmaps...');
k = 1;  % Choose the time step index to visualize
for ti = 1:num_theta
    V_slice = squeeze(V(k, :, :, ti));
    figure;
    imagesc(x_grid, y_grid, V_slice);
    set(gca, 'YDir', 'normal');
    colorbar;
    xlabel('x');
    ylabel('y');
    title(sprintf('Value Function V at time step %d, θ index = %d', k, ti));
    % saveas(gcf, sprintf('V_heatmap_k%d_theta%d.png', k, ti));
    % close;
end
disp('Value Function heatmaps plotted and saved.');

%% ------------------------------
% Simulation
% ------------------------------
disp('Starting simulation...');
% Initial condition
initial_state = [0.5; 0.5; 0.0];  % [x; y; theta]
update_state = initial_state;

% Initialize trajectory and inputs
trajectory = update_state;
omegas = [];
speeds = [];

% Run simulation
for t_index = 1:time_steps
    x = update_state(1);
    y = update_state(2);
    theta = update_state(3);
    
    % Get indices for the current state
    [x_index, y_index, theta_index] = get_indices(update_state, x_grid, y_grid, theta_grid);
    
    % Retrieve the best action from the policy
    action = squeeze(policy(t_index, x_index, y_index, theta_index, :));
    omega = action(1);
    v = action(2);
    
    % Store the actions
    omegas = [omegas; omega];
    speeds = [speeds; v];
    
    % Compute the next state using the best action
    update_state = dynamics(update_state, action, delta_t);
    trajectory = [trajectory, update_state];
end
disp('Simulation completed.');

%% ------------------------------
% Prepare Time Vector
% ------------------------------
input_time = (0:length(omegas)-1)' * delta_t;  % Column vector

%% ------------------------------
% Extract Trajectory Data
% ------------------------------
x_traj = trajectory(1, :);
y_traj = trajectory(2, :);

%% ------------------------------
% Define Boundary for Visualization
% ------------------------------
boundary_x = [-1, 1, 1, -1, -1];  % Close the boundary rectangle
boundary_y = [-1, -1, 1, 1, -1];

%% ------------------------------
% Plotting
% ------------------------------
disp('Generating plots...');

% 1. Plot omega (input) over time
figure;
plot(input_time, omegas, 'b', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Omega (rad/s)');
title('Omega Over Time');
legend('Omega');
grid on;
saveas(gcf, 'Omega_Over_Time.png');
% close;

% 2. Plot speed v over time
figure;
plot(input_time, speeds, 'g', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Speed (units/s)');
title('Speed Over Time');
legend('Speed');
grid on;
saveas(gcf, 'Speed_Over_Time.png');
% close;

% 3. Plot trajectory with boundary and initial condition
figure;
plot(x_traj, y_traj, 'b-', 'LineWidth', 1.5);
hold on;
plot(boundary_x, boundary_y, 'r-', 'LineWidth', 2);
scatter(x_traj(1), y_traj(1), 100, 'k', 'filled');  % Initial condition
xlabel('x');
ylabel('y');
title('Trajectory with Boundary');
legend('Trajectory', 'Boundary', 'Initial Condition');
axis equal;
grid on;
saveas(gcf, 'Trajectory_with_Boundary.png');
% close;

% Combine all three plots into a single figure
figure;
subplot(3,1,1);
plot(input_time, omegas, 'b', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Omega (rad/s)');
title('Omega Over Time');
legend('Omega');
grid on;

subplot(3,1,2);
plot(input_time, speeds, 'g', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Speed (units/s)');
title('Speed Over Time');
legend('Speed');
grid on;

subplot(3,1,3);
plot(x_traj, y_traj, 'b-', 'LineWidth', 1.5);
hold on;
plot(boundary_x, boundary_y, 'r-', 'LineWidth', 2);
scatter(x_traj(1), y_traj(1), 100, 'k', 'filled');  % Initial condition
xlabel('x');
ylabel('y');
title('Trajectory with Boundary');
legend('Trajectory', 'Boundary', 'Initial Condition');
axis equal;
grid on;

% % Save the combined plot
% saveas(gcf, 'trajectory_inputs_speeds_plot.png');
% close;
% disp('Plots generated and saved as "trajectory_inputs_speeds_plot.png".');
% 
% disp('All tasks completed successfully.');

% close all