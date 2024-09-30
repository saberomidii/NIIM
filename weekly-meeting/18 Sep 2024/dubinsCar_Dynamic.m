clc;
clear;

dt = 0.01;            % Time step (seconds)
% Desired trajectory function (scaled quadratic curve)
desired_trajectory = @(x) 0.75 * x.^2; % Scaled to fit (2,2) target

% Offset trajectories (feasible set boundaries)
offset_positive = @(x) desired_trajectory(x) + 0.2;
offset_negative = @(x) desired_trajectory(x) - 0.2;
% Define the range for x and y
x_range = -0.5:dt:2;
y_range = -0.5:dt:3.5;

% Initialize sets
set_in = [];
set_out = [];

% Loop through all points in the grid
for x = x_range
    for y = y_range
        % Calculate the feasible set boundaries for the current x
        upper_bound = offset_positive(x);
        lower_bound = offset_negative(x);
        
        % Check if the point is inside or outside the feasible bounds
        if y >= lower_bound && y <= upper_bound
            % Point is inside the bounds
            set_in = [set_in; x, y];
        else
            % Point is outside the bounds
            set_out = [set_out; x, y];
        end
    end
end

% Simulation parameters
total_time = 19;     % Total time for simulation (seconds)
steps = total_time / dt; % Number of steps

% Car parameters
v = 0.3;             % Constant forward speed

% PID Controller parameters
Kp = 2;              % Proportional gain
Ki = 0.05;           % Integral gain
Kd = 1;              % Derivative gain

% Initial state [x; y; theta]
x = 0.1;               % Initial x position
y = 0.1;               % Initial y position
theta = 0;           % Initial orientation

% PID controller variables
integral_error = 0;
previous_error = 0;

% Arrays to store state values for plotting
x_values = zeros(steps, 1);
y_values = zeros(steps, 1);

% Arrays to store signed distance values
epsilon = 0.01;
U_min = -0.5;
U_max = 0.5;
kappa_V(1) = 0;
V_x(1) = 0.2;
omega(1) = 0;

% Simulate the Dubins car dynamics
for i = 1:steps
    % Store the current position
    x_values(i) = x;
    y_values(i) = y;
    
    % Calculate the desired y position and the error
    y_desired = desired_trajectory(x);
    error = y_desired - y;
    
    %%% switching part 
    % Implement the switching logic
    % if V_x(i) <= epsilon || omega(i) < U_min || omega(i) > U_max
    %     omega(i+1) = kappa_V(1);
    % % else 
    % PID control for angular velocity (omega)
        integral_error = integral_error + error * dt;
        derivative_error = (error - previous_error) / dt;
        omega(i) = Kp * error + Ki * integral_error + Kd * derivative_error;
    % end 
    
    %%% state constraints 
    % Calculate the feasible set boundaries for the current x
    upper_bound = offset_positive(x);
    lower_bound = offset_negative(x);
        
    % Check if the point is inside or outside the feasible bounds
    if y >= lower_bound && y <= upper_bound  %% inside X 
        signed_distance(i) = max(vecnorm([x, y] - set_in, 2, 2));
        cost_functional(i) = min(-signed_distance);
        V_x(i+1) = max(cost_functional);
        hold on
        plot(x, y, "o", 'Color', 'blue')

    else
        signed_distance(i) = min(vecnorm([x, y] - set_out, 2, 2));
        cost_functional(i) = -min(signed_distance);
        V_x(i+1) = max(cost_functional);
        hold on
        plot(x, y, "x", 'Color', 'red')
    end

    % Stop if x exceeds the goal (x, y) = (2, 2)
    if x >= 2 && y >= 2
        x_values = x_values(1:i);
        y_values = y_values(1:i);
        time_values = time_values(1:i);
        break;
    end

    %%%% Find u to maximize the gradient of V_x times dynamics
    grad_v = gradient(V_x);
    
    % Iterate over possible values of u to find maximum
    u_range = U_min:0.01:U_max; % Resolution can be adjusted
    max_value = -Inf;
    optimal_u = 0;

    for u = u_range
        % Dynamics of the system (example dynamics for the Dubins car)
        dynamics = [v * cos(theta); v * sin(theta); u];
        
        % Evaluate the product of the gradient and the dynamics
        value = grad_v(i) * dynamics;

        % Check if this is the maximum value found so far
        if value > max_value
            max_value = value;
            kappa_V(i) = u;
        end
    end

    % % Use the optimal control input to update the system
    % omega(i+1) = kappa_V(i);

    % Update states using the Dubins car model equations
    x = x + v * cos(theta) * dt;
    y = y + v * sin(theta) * dt;
    theta = theta + omega(i) * dt;

end

hold on;
% Plot the desired trajectory
fplot(desired_trajectory, [0, 2], 'g--', 'LineWidth', 1.5);
% Plot the offset trajectories
fplot(offset_positive, [0, 2], 'r--', 'LineWidth', 1);
fplot(offset_negative, [0, 2], 'r--', 'LineWidth', 1);

xlabel('X (meters)');
ylabel('Y (meters)');
title('Dubins Car Trajectory with PID Control and Feasible Set Boundaries');
grid on;
hold off;

figure;
plot(signed_distance, 'b-', 'LineWidth', 1.5); % Plot the signed distances in blue with a line width of 1.5

% Add title and labels
title('Signed Distance to the Feasible Set Over Time');
xlabel('Time Step');
ylabel('Signed Distance');

% Add grid for better readability
grid on;

% Add legend
legend('Signed Distance');

% Display the plot
hold off;

figure;
plot(cost_functional, 'b-', 'LineWidth', 1.5); % Plot the signed distances in blue with a line width of 1.5

% Add title and labels
title('Cost Functional');
xlabel('Time Step');
ylabel('Cost Functional');

% Add grid for better readability
grid on;

% Add legend
legend('Cost Functional');

% Display the plot
hold off;

figure;
plot(V_x, 'b-', 'LineWidth', 1.5); % Plot the signed distances in blue with a line width of 1.5

% Add title and labels
title('Value Function');
xlabel('Time Step');
ylabel('Value Function');

% Add grid for better readability
grid on;

% Add legend
legend('Value Function');

% Display the plot
hold off;
