% Clear workspace, command window, and close all figures
clear;
clc;
close all;

% Define minimum and maximum values as vectors
min_vals = [-5; -3];
max_vals = [3; 5];

% Define the feasible set
feasible_set = define_rectangular_set(min_vals, max_vals);

% Simulation parameters
dt = 0.01;                % Time step (set to 0.01 as per your requirement)
Last_step = 50;           % Number of simulation steps
v = 1;                    % Constant speed

% Create input and state lists with a step of 0.01
input_list = -1:dt:1;     % Inputs from -1 to 1 with 0.01 step (201 elements)
state_1_list = -5:dt:-3;  % State1 from -5 to -3 with 0.01 step (201 elements)
state_2_list = 3:dt:5;    % State2 from 3 to 5 with 0.01 step (201 elements)

% Determine the number of elements in each list
num_inputs = length(input_list);
num_state_1 = length(state_1_list);
num_state_2 = length(state_2_list);

% Calculate total number of combinations
total_combinations = num_inputs * num_state_1 * num_state_2;

% Preallocate arrays to store data
u_data = zeros(total_combinations, 1);      % Input values
x1_data = zeros(total_combinations, 1);     % State1 values
x2_data = zeros(total_combinations, 1);     % State2 values
l_data = zeros(total_combinations, 1);      % L values

% Initialize a counter for data storage
data_idx = 1;

% Start timing the simulation
tic;

% Loop through each State2 value
for s2_idx = 1:num_state_2
    state_2 = state_2_list(s2_idx);
    
    % Loop through each State1 value
    for s1_idx = 1:num_state_1
        state_1 = state_1_list(s1_idx);
        
        % Loop through each Input value
        for u_idx = 1:num_inputs
            input = input_list(u_idx);
            adversial_disturbance=rand;
            % Initialize state [x; y; theta]
            state = [state_1; state_2; 0];
            
            % Initialize state history
            state_history = state;
            adversial_disturbance_history=adversial_disturbance;
            % Initialize minimum l value for this trajectory
            min_l = Inf;
            
            % Simulate the trajectory for Last_step time steps
            for step = 1:Last_step
                % Update state using Dubin's car dynamics
                state = dubins_car(state, input, v, adversial_disturbance, dt);
                
                % Append the new state to the state history
                state_history = [state_history, state];
                
                % Compute the signed distance for the current position
                s = signed_distance(state_history(1:2, end), min_vals, max_vals);
                
                % Compute l = -s
                l = -s;
                
                % Update the minimum l value
                if l < min_l
                    min_l = l;
                end
            end
            
            % Store the results in the preallocated arrays
            u_data(data_idx) = input;
            x1_data(data_idx) = state_1;
            x2_data(data_idx) = state_2;
            l_data(data_idx) = min_l;
            
            % Increment the data index
            data_idx = data_idx + 1;
        end
    end
end

% End timing the simulation
toc;

% Create a MATLAB table with the collected data
data_table = table(u_data, x1_data, x2_data, l_data, ...
                   'VariableNames', {'Input', 'State1', 'State2', 'L_Value'});

% Display the first few rows of the table to verify
disp('Sample of the Data Table:');
disp(head(data_table));

% Optional: Save the table to a CSV file for further analysis
writetable(data_table, 'simulation_results.csv');

% Function Definitions






% Plot the feasible set and the optimal trajectory
figure;
hold on;
grid on;

% Plot the rectangle representing the feasible set
rectangle('Position', [min_vals(1), min_vals(2), ...
    max_vals(1)-min_vals(1), max_vals(2)-min_vals(2)], ...
    'EdgeColor', 'r', 'LineWidth', 2, 'DisplayName', 'Feasible Set');

% Plot the optimal trajectory
plot(state_history_opt(1, :), state_history_opt(2, :), 'b-o', ...
    'LineWidth', 2, 'MarkerFaceColor', 'b', 'DisplayName', 'Optimal Trajectory');

% Highlight the starting point
plot(state_history_opt(1, 1), state_history_opt(2, 1), 'go', 'MarkerSize', 10, ...
    'MarkerFaceColor', 'g', 'DisplayName', 'Start Point');

% Highlight the ending point
plot(state_history_opt(1, end), state_history_opt(2, end), 'mo', 'MarkerSize', 10, ...
    'MarkerFaceColor', 'm', 'DisplayName', 'End Point');

% Labels and Title
xlabel('x');
ylabel('y');
title('Optimal Trajectory within Feasible Set');
legend('Location', 'best');
axis equal;
hold off;





function next_state = dubins_car(state, input,adversial_disturbance, v, dt)
    % Simple second-order dynamic function for Dubin's car
    % state: [x; y; theta]
    % input: omega (steering rate)
    % v: constant speed
    % dt: time step
    % output: next_state

    x_next = state(1) + v * cos(state(3)) * dt;
    y_next = state(2) + v * sin(state(3)) * dt;
    theta_next = state(3) + (input+adversial_disturbance) * dt;

    next_state = [x_next; y_next; theta_next];
end

function rect_set = define_rectangular_set(min_vals, max_vals)
    % Function to define a rectangular set given min and max vectors
    % min_vals: vector of minimum values [x_min; y_min]
    % max_vals: vector of maximum values [x_max; y_max]
    % rect_set: structure containing min and max values for each dimension

    % Check that min_vals and max_vals are the same size
    if length(min_vals) ~= length(max_vals)
        error('min_vals and max_vals must be vectors of the same length');
    end

    % Create a structure to hold the rectangular set
    rect_set.min = min_vals;
    rect_set.max = max_vals;
end

function s = signed_distance(position, min_vals, max_vals)
    % signed_distance Computes the signed distance for a single point
    %
    %   s = signed_distance(position, min_vals, max_vals)
    %
    %   Inputs:
    %       position - 2x1 vector [x; y]
    %       min_vals - 2x1 vector [x_min; y_min] defining the rectangular set
    %       max_vals - 2x1 vector [x_max; y_max] defining the rectangular set
    %
    %   Output:
    %       s - scalar signed distance value
    %
    %   Notes:
    %       - Negative value indicates point inside the set
    %       - Positive value indicates point outside the set

    x = position(1);
    y = position(2);

    if (x >= min_vals(1)) && (x <= max_vals(1)) && ...
       (y >= min_vals(2)) && (y <= max_vals(2))
        % Inside the set

        % Compute distances to the boundaries
        distance_to_min_x = x - min_vals(1);
        distance_to_max_x = max_vals(1) - x;
        distance_to_min_y = y - min_vals(2);
        distance_to_max_y = max_vals(2) - y;

        % Find the minimum distance to any boundary
        min_distance_to_boundary = min([distance_to_min_x, distance_to_max_x, ...
                                       distance_to_min_y, distance_to_max_y]);

        % Signed distance is negative of the minimum distance to boundary
        s = -min_distance_to_boundary;
    else
        % Outside the set

        % Compute distance in x-direction
        if x < min_vals(1)
            dx = min_vals(1) - x;
        elseif x > max_vals(1)
            dx = x - max_vals(1);
        else
            dx = 0;
        end

        % Compute distance in y-direction
        if y < min_vals(2)
            dy = min_vals(2) - y;
        elseif y > max_vals(2)
            dy = y - max_vals(2);
        else
            dy = 0;
        end

        % Compute the Euclidean distance to the set
        s = sqrt(dx^2 + dy^2);
    end
end

function l = l_function(s)
    % l_function Computes the negative signed distance
    %
    %   l = l_function(s)
    %
    %   Inputs:
    %       s - scalar signed distance
    %
    %   Output:
    %       l - scalar value (negative signed distance)

    l = -s;
end
