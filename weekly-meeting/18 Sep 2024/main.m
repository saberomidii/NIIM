clc;
clear;
close all

%%% Desired trajectory with bounds
% Desired trajectory function (scaled quadratic curve)
desired_trajectory = @(x) 0.75 * x.^2; % Scaled to fit (2,2) target

% Offset trajectories (feasible set boundaries)
offset_positive = @(x) desired_trajectory(x) + 0.2;
offset_negative = @(x) desired_trajectory(x) - 0.2;

% Define the range for x, y, and theta
dx = 0.1;           % Step size for x (adjusted for performance)
dy = 0.1;           % Step size for y
dtheta = 0.1;       % Step size for theta

x_range = 0:dx:2;
y_range = -1:dy:3.5; % Adjusted to include points outside the feasible set
theta_min = -0.5;
theta_max = 0.5;
theta_range = theta_min:dtheta:theta_max;

% Create meshgrid for x, y, theta
[XX, YY, TT] = meshgrid(x_range, y_range, theta_range);

% Calculate upper and lower bounds for y
Upper_Bound = offset_positive(XX);
Lower_Bound = offset_negative(XX);

% Define the function F(x, y, theta)
% F is negative inside the safe set and positive outside
F = max(max(YY - Upper_Bound, Lower_Bound - YY), max(TT - theta_max, theta_min - TT));

% Determine if points are inside or outside the feasible set
inside_mask = (YY >= Lower_Bound) & (YY <= Upper_Bound) & (TT >= theta_min) & (TT <= theta_max);

% Extract coordinates of safe and unsafe points
X_safe = XX(inside_mask);
Y_safe = YY(inside_mask);
Theta_safe = TT(inside_mask);

X_unsafe = XX(~inside_mask);
Y_unsafe = YY(~inside_mask);
Theta_unsafe = TT(~inside_mask);

% Create figure
figure;

% Plot the safe set as an isosurface where F = 0
p_safe = patch(isosurface(XX, YY, TT, F, 0));
isonormals(XX, YY, TT, F, p_safe);
p_safe.FaceColor = 'green';
p_safe.EdgeColor = 'none';
p_safe.FaceAlpha = 0.2; % Adjust transparency

hold on;

% Plot the unsafe set boundary as an isosurface at a small positive value
p_unsafe = patch(isosurface(XX, YY, TT, F, 0.01));
isonormals(XX, YY, TT, F, p_unsafe);
p_unsafe.FaceColor = 'red';
p_unsafe.EdgeColor = 'none';
p_unsafe.FaceAlpha = 0.2;

% Plot the safe points with green circles
h_safe_points = scatter3(X_safe, Y_safe, Theta_safe, 20, 'o', 'MarkerEdgeColor', 'green', 'MarkerFaceColor', 'green');

% Plot the unsafe points with red crosses
h_unsafe_points = scatter3(X_unsafe, Y_unsafe, Theta_unsafe, 20, 'x', 'MarkerEdgeColor', 'red');

% Set labels and title
xlabel('x');
ylabel('y');
zlabel('\theta');
title('Feasible (Green) and Unsafe (Red) Sets with Surfaces and Points');
grid on;
view(3);
axis tight;
camlight;
lighting gouraud;

% Add legend
legend([p_safe, p_unsafe, h_safe_points, h_unsafe_points], {'Feasible Set Surface', 'Unsafe Set Surface', 'Safe Points', 'Unsafe Points'}, 'Location', 'best');

% Enable 3D rotation
rotate3d on;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%clc;
clear;
close all

%%% Desired trajectory with bounds
% Desired trajectory function (scaled quadratic curve)
desired_trajectory = @(x) 0.75 * x.^2; % Scaled to fit (2,2) target

% Offset trajectories (feasible set boundaries)
offset_positive = @(x) desired_trajectory(x) + 0.2;
offset_negative = @(x) desired_trajectory(x) - 0.2;

% Define the range for x and y
dt = 0.2;            % Step size
x_range = 0:dt:2;
y_range = -1:dt:3.5; % Adjusted to include points outside the feasible set

% Define the theta range
theta_min = -0.5;
theta_max = 0.5;
dt_theta = 0.1; % Step size for theta (adjust as needed)
theta_range = theta_min:dt_theta:theta_max;

% Initialize sets
set_in = [];
set_out = [];

% Create set_in and set_out as points including theta
for x = x_range
    lower_bound = offset_negative(x);
    upper_bound = offset_positive(x);
    for y = y_range
        for theta = theta_range
            if y >= lower_bound && y <= upper_bound && theta >= theta_min && theta <= theta_max
                % Point is inside the bounds (safe set)
                set_in = [set_in; x, y, theta];
            else
                % Point is outside the bounds (unsafe set)
                set_out = [set_out; x, y, theta];
            end
        end
    end
end

% Define points along the desired trajectory
dt=0.01;
x_values = 0:dt:2;         % x-values from 0 to 2 with step size dt
y_values = 0.5*desired_trajectory(x_values);
theta_values = zeros(size(x_values)); % Assuming theta = 0 along the trajectory


% Plot the desired trajectory
figure;
hold on
plot(x_values, y_values, 'b-', 'LineWidth', 2);
xlabel('x');
ylabel('y');
title('Desired Trajectory');
grid on;
plot(set_in(:,1), set_in(:,2), 'o', 'Color', 'green', 'DisplayName', 'Feasible');
plot(set_out(:,1), set_out(:,2), 'x', 'Color', 'red', 'DisplayName', 'Unsafe');
legend('show'," Desired Trajcotry","Feasible set ","Unsafe set");


% Initialize array for signed distance function values
signed_distance_function_values = zeros(size(x_values));



% Initialize GIF parameters
gifFilename = 'SignedDistanceAnimation.gif';
delayTime = 0.1; % Delay time between frames in seconds
firstFrame = true;

% Loop over points and compute signed distance
for index = 1:length(x_values)
    x = x_values(index);
    y = y_values(index);
    theta = theta_values(index);
    show_plot = true; % Set to true to display and save plots
    
    % Call the modified signed_distance_function
    signed_distance(index) = signed_distance_function(x, y, theta, set_in, set_out, show_plot, gifFilename, firstFrame, delayTime);
    
    % Update firstFrame flag after first iteration
    if firstFrame
        firstFrame = false;
    end
    
    % Optionally, pause or adjust as needed
    pause(0.1); % Adjust the pause duration to control the speed of the animation
end

% Compute the cost function as the negative of the signed distance
cost_function_values = -signed_distance;

% Find the supremum (maximum value) of the cost function
supremum_cost = min(cost_function_values);

% Plot the cost function over time steps
figure;
plot(x_values, cost_function_values, 'b-', 'LineWidth', 2);
hold on;

% Plot the supremum as a green horizontal line
yline(supremum_cost, 'g--', 'LineWidth', 2);

% Add labels and title
xlabel('Time Step');
ylabel('Cost Function');
title('Cost Function over Time Steps');


% Add legend
legend('Cost Function', 'Inf -SDF', 'Location', 'best');

grid on;
hold off;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%closed loop 
%%
clc;
clear;
close all;

% Load data if needed (commented out since files are not provided)
load("uOpt.mat",'uOpt')
load('g.mat', 'g');
load('V.mat', 'V');


%%% Desired trajectory with bounds
desired_trajectory = @(x) 0.75 * x.^2; % Scaled to fit (2,2) target
offset_positive = @(x) desired_trajectory(x) + 0.2;
offset_negative = @(x) desired_trajectory(x) - 0.2;

% Define the range for x and y
dt = 0.1;            % Reduced time step (seconds)
x_range = 0:dt:2;
y_range = 0:dt:3.5;

% Define the theta range
theta_min = -0.5;
theta_max = 0.5;
theta_range = theta_min:dt:theta_max;
theta_values = zeros(size(x_range)); % Assuming theta = 0 along the trajectory

% Initialize sets
set_in = [];
set_out = [];

% Create set_in and set_out as points including theta
for x = x_range
    lower_bound = offset_negative(x);
    upper_bound = offset_positive(x);
    for y = y_range
        for theta = theta_range
            if y >= lower_bound && y <= upper_bound && theta >= theta_min && theta <= theta_max
                % Point is inside the bounds (safe set)
                set_in = [set_in; x, y, theta];
            else
                % Point is outside the bounds (unsafe set)
                set_out = [set_out; x, y, theta];
            end
        end
    end
end

figure;
plot(set_in(:,1), set_in(:,2), 'o', 'Color', 'green', 'DisplayName', 'Feasible');
hold on;
plot(set_out(:,1), set_out(:,2), 'x', 'Color', 'red', 'DisplayName', 'Unsafe');

% Adding labels and title
xlabel('X Position');
ylabel('Y Position');
title('Feasible and Unsafe Sets');
grid on;

%%% Initial conditions
x0 = 0;
y0 = 0;
theta0 = 0; % Initial orientation aligned with the desired path

omega_min=-1;
omega_max=1;
plot(x0, y0, 'x', 'Color', 'black', 'DisplayName', 'Initial Condition');

%%% Sample trajectory (desired)
x_values = 0:dt:2;         % x-values from 0 to 2 with step size dt
y_values = desired_trajectory(x_values);
plot(x_values, y_values, 'blue', 'LineWidth', 2, 'DisplayName', 'Desired trajectory');

%%%%%%%%%%%%%%%%%%%%%%%% Sliding Mode Control for Dubin's Car %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Car parameters
v = 0.2;             % Constant forward speed


% Simulation parameters
total_time = 19;     % Total time for simulation (seconds)
steps = total_time / dt; % Number of steps

% Sliding Mode Controller parameters
lambda = 5;          % Increased slope of the sliding surface
eta = 1;             % Gain for the switching component

% Initial state [x; y; theta]
x = x0;               % Initial x position
y = y0;               % Initial y position
theta = theta0;       % Initial orientation

% Arrays to store state values for plotting
x_values_actual = zeros(steps, 1);
y_values_actual = zeros(steps, 1);
theta_values = zeros(steps, 1);
omega = zeros(steps, 1);
epsilon=0.001;
% Simulate the Dubins car dynamics
for i = 1:steps
    
    % Store the current position
    x_values_actual(i) = x;
    y_values_actual(i) = y;
    theta_values(i) = theta;
    
    y_desired = desired_trajectory(x);

    %%% another controller 
    omega(i)=-[1,10,1]*([x;y;theta]-[0;y_desired;0]);
    

    %%% Sigined distance function and cost function
    singed_distance_function_values(i)=signed_distance_function(x, y, theta, set_in, set_out, false, [], [], []);
    cost_function(i)=min(-singed_distance_function_values);
    value_at_state= interpn(g.xs{1}, g.xs{2}, g.xs{3}, V, x, y, theta);
    value_at_time(i)=value_at_state(:,:,:,end);
    % [Value,optimal_input]=computeValueFunction(x, y, theta, i);
    % optimal_input
    % omega(i)=optimal_input;
    if isnan(value_at_state(i))
             display("break, out of state sapce set")
            break;    
    end

    if singed_distance_function_values(i) > 0  || omega(i) < omega_min || omega(i) > omega_max


             display("Safety Violation") 
             plot(x, y, 'x', 'Color', '#ad56dc')
             % display(theta)
             input_at_state=interpn(g.xs{1}, g.xs{2}, g.xs{3},uOpt,x, y, theta);
             omega(i)=-input_at_state(:,:,:,end);
             % omega(i)=-[1,19,1]*([x;y;theta]-[0;y_desired;0]);

             % Calculate the desired y position and the error
            y_desired = desired_trajectory(x);
            e_y = y_desired - y;
            
            % Compute derivative of the desired trajectory
            dy_desired_dx = 1.5 * x; % Since desired_trajectory = 0.75 * x^2
            
            % Compute y_desired_dot
            y_desired_dot = dy_desired_dx * v * cos(theta);
            
            % Compute actual y_dot
            y_dot = v * sin(theta);
            
            % Corrected error derivative
            e_y_dot = y_desired_dot - y_dot;
            
            % Sliding surface
            s = e_y + lambda * e_y_dot;
            
            % Sliding Mode Control law
            omega(i) = lambda * e_y_dot + eta * sign(s);
            
            % Saturate control input to maximum allowable angular velocity
            omega(i) = max(min(omega(i), 1), -1);
            % Optional: Ensure theta stays within -pi to pi
            % if theta < theta_min 
            %     theta=theta_min;
            % elseif  theta > theta_max
            %     theta=theta_max;
            % end
            % 
    
    end


    % Update states using the Dubins car model equations
    x = x + v * cos(theta) * dt;
    y = y + v * sin(theta) * dt;
    theta = theta + omega(i) * dt;
    
    % Optional: Ensure theta stays within -pi to pi
    theta = mod(theta + pi, 2*pi) - pi;
    
    % Debugging: Display control values (commented out for cleaner output)
    % fprintf('Step: %d, X: %.2f, Y: %.2f, Theta: %.2f, Omega: %.2f, Error: %.2f\n', i, x, y, theta, omega(i), e_y);
end

% Plot the actual trajectory
plot(x_values_actual, y_values_actual, 'Color', 'yellow', 'LineWidth', 2, 'DisplayName', 'Actual trajectory');
legend('show',"Feasible set","Unsafe set","Initial condition","Desired Trajcotry","SM input","Opt input");

% Plotting omega over time steps
figure;
plot(omega, 'b-', 'LineWidth', 2);
xlabel('Time Step');
ylabel('Omega (Input)');
title('Omega Input over Time Steps');
grid on;







%%% trash 
% 

% % Adding labels and title
% xlabel('X Position');
% ylabel('Y Position');
% title('Feasible and Unsafe Sets');
% 
% grid on;
% 
% 
% %%% initial conditions
% x0=0;
% y0=0;
% plot(x0,y0,"x",'Color',"black")
% 
% %%% Sample trajectory (desired)
% x_values = 0:dt:2;         % x-values from -0.5 to 2 with step size dt
% y_values = desired_trajectory(x_values);
% 
% plot(x_values,y_values,"blue",'LineWidth',2)
% 
% % for index=1:length(x_values)
% % singed_distance_function_values(index)=signed_distance_function(x_values(index), y_values(index), set_in, set_out,true);
% % costal_function(index)=min(-singed_distance_function_values);
% % value_function(index)=max(costal_function);
% % end
% 
% % figure 
% % plot(costal_function)
% % 
% % figure 
% % plot3(x_values,y_values,value_function)
% %%%%%%%%%%%%%%%%%%%%%%% PID for Dubin's Car %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Car parameters
% v = 0.2;             % Constant forward speed
% % Simulation parameters
% total_time = 19;     % Total time for simulation (seconds)
% steps = total_time / dt; % Number of steps
% 
% % PID Controller parameters
% Kp = 4;              % Proportional gain
% Ki = 0.5;             % Integral gain
% Kd = 0.01;               % Derivative gain
% 
% % Initial state [x; y; theta]
% x = 0;               % Initial x position
% y = 0;               % Initial y position
% theta = 0;           % Initial orientation
% 
% % PID controller variables
% integral_error = 0;
% previous_error = 0;
% 
% % Arrays to store state values for plotting
% x_values = zeros(steps, 1);
% y_values = zeros(steps, 1);
% theta_values= zeros(steps, 1);
% value_at_state=zeros(steps, 1);
% value_at_state(1) = interpn(g.xs{1}, g.xs{2}, g.xs{3}, V_final, x, y, theta);
% 
% 
% % Simulate the Dubins car dynamics
% for i = 1:steps
%     % Store the current position
%     x_values(i) = x;
%     y_values(i) = y;
%     theta_values(i)=theta;
% 
%     % value_function(i)=max(cost_function);
%     % Calculate the desired y position and the error
%     y_desired = desired_trajectory(x);
%     error = y_desired - y;
% 
%     %PID
%         integral_error = integral_error + error * dt;
%         derivative_error = (error - previous_error) / dt;
%         omega(i) = Kp * error + Ki * integral_error + Kd * derivative_error;
% 
% 
%     %%% Sigined distance function and cost function
%     singed_distance_function_values(i)=signed_distance_function(x, y, set_in, set_out,false);
%     cost_function(i)=min(-singed_distance_function_values);
%     value_at_state(i) = interpn(g.xs{1}, g.xs{2}, g.xs{3}, V_final, x, y, theta);
% 
%     if isnan(value_at_state(i))
%              display("break, out of state sapce set")
%             break;    
%     end
% 
%     if value_at_state(i) < 0 || theta < theta_min || theta > theta_max
%              % display("Safet Violation") 
%              % plot(x,y,"x",'Color',"magenta")
%              % display(theta)
%              omega(i)=u_opt(i);
%     end
% 
%     % Update states using the Dubins car model equations
%     x = x + v * cos(theta) * dt;
%     y = y + v * sin(theta) * dt;
%     theta = theta + omega(i) * dt;
% 
% end
% plot(x_values,y_values,'Color',"yellow",LineWidth=2)
% legend('show',"Feasible","Unsafe","Initial Condition","Desired trajecotry","Actual trajectory","safety violation");
% figure 
% plot(cost_function)

% figure 
% plot(value_function)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Create a mesh grid for x and y
% [X, Y] = meshgrid(x_range, y_range);
% J = ones(size(X));
% 
% for i = 1:size(X, 1)  % Iterate over rows (related to Y values)
%     for j = 1:size(X, 2)  % Iterate over columns (related to X values)
%         J(i,j)=-signed_distance_function(X(i,j), Y(i,j), set_in, set_out,false);
%     end
% end
% 
% % Plotting the grid
% figure;
% mesh(X, Y, J); % Create a 3D mesh plot
% xlabel('x'); % Label for x-axis
% ylabel('y'); % Label for y-axis
% zlabel('z'); % Label for z-axis
% title('3D Mesh Grid with Correct Z-values');
% grid on; % Enable the grid for better visualization





% wMax=1;
% % Assuming the necessary variables and functions are already defined and loaded:
% % 'g', 'V_final', 'dCar', 'gradient', and 'interpn' functions
% 
% % Compute the spatial gradients of V_final
% [gradV_x, gradV_y, gradV_theta] = gradient(-V_final, g.dx(1), g.dx(2), g.dx(3));
% 
% % Initialize arrays to store control inputs
% u_opt = zeros(steps, 1);
% 
% % Loop through the simulation steps
% for i = 1:steps
%     % Evaluate the gradient at the current state [x, y, theta]
%     gradV_x_val = interpn(g.xs{1}, g.xs{2}, g.xs{3}, gradV_x, x, y, theta);
%     gradV_y_val = interpn(g.xs{1}, g.xs{2}, g.xs{3}, gradV_y, x, y, theta);
%     gradV_theta_val = interpn(g.xs{1}, g.xs{2}, g.xs{3}, gradV_theta, x, y, theta);
% 
%     % Gradient vector
%     gradV = [gradV_x_val; gradV_y_val; gradV_theta_val];
% 
%     % Dynamics of the Dubins car: [x_dot; y_dot; theta_dot]
%     % x_dot = v * cos(theta)
%     % y_dot = v * sin(theta)
%     % theta_dot = u (control input to be determined)
% 
%     % Define an anonymous function for the dot product
%     dot_product = @(u) gradV' * [v * cos(theta); v * sin(theta); u];
% 
%     % Find the control input that maximizes the dot product
%     % Since u can take values in [-wMax, wMax], we can find the optimal control
%     u_opt(i) = fminbnd(@(u) -dot_product(u), -wMax, wMax);  % Using fminbnd to find maximum
% 
%     % Update states using the Dubins car model equations
%     x = x + v * cos(theta) * dt;
%     y = y + v * sin(theta) * dt;
%     theta = theta + u_opt(i) * dt;
% end
% 
% save('u_opt.mat', 'u_opt');
% 
% % Plot the optimal control input over time
% figure;
% plot((0:steps-1)*dt, u_opt, 'LineWidth', 2);
% xlabel('Time (s)');
% ylabel('Optimal Control Input \omega');
% title('Optimal Control Input \omega over Time');
% grid on;
% 