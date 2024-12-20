% clear all
% clc
% close all
% % Define the limits for x1 and x2 (same as the rectangle)
% x1_min = 0;
% x1_max = 4;
% x2_min = 0;
% x2_max = 3;
% 
% % Create a grid of points over a specified range
% [X1, X2] = meshgrid(linspace(x1_min - 2, x1_max + 2, 200), linspace(x2_min - 2, x2_max + 2, 200));
% 
% % Initialize the signed distance function s_M(z)
% sM = zeros(size(X1));
% 
% % Compute s_M(z) at each grid point
% for i = 1:size(X1, 1)
%     for j = 1:size(X1, 2)
%         z1 = X1(i, j);
%         z2 = X2(i, j);
% 
%         % Check if the point is inside the rectangle
%         inside_x = (z1 >= x1_min) && (z1 <= x1_max);
%         inside_y = (z2 >= x2_min) && (z2 <= x2_max);
% 
%         if inside_x && inside_y
%             % Point is inside the rectangle
%             d_left   = z1 - x1_min;
%             d_right  = x1_max - z1;
%             d_bottom = z2 - x2_min;
%             d_top    = x2_max - z2;
%             d_min    = min([d_left, d_right, d_bottom, d_top]);
%             sM(i, j) = -d_min;
%         else
%             % Point is outside the rectangle
%             dx = max([x1_min - z1, 0, z1 - x1_max]);
%             dy = max([x2_min - z2, 0, z2 - x2_max]);
%             sM(i, j) = sqrt(dx^2 + dy^2);
%         end
%     end
% end
% 
% % Compute l = -s_M(z)
% l = -sM;
% 
% % Plot l as a 3D surface plot
% figure;
% 
% % Use surf to create a 3D surface plot
% surf(X1, X2, l, 'EdgeColor', 'none');
% 
% % Add labels and title
% xlabel('x1');
% ylabel('x2');
% zlabel('l = -s_M(z)');
% title('3D Surface Plot of l = -s_M(z)');
% 
% % Adjust the view angle for better visualization
% view(45, 30); % Adjust the azimuth and elevation angles
% 
% % Add lighting and shading effects for better visualization
% shading interp; % Interpolate colors across lines and faces
% camlight left;  % Add a light source
% lighting phong; % Use Phong lighting model
% 
% % Adjust the axes limits
% xlim([x1_min - 2, x1_max + 2]);
% ylim([x2_min - 2, x2_max + 2]);
% grid on;
% 

% ============================
% Updated MATLAB Script
% ============================

%% Clear workspace and figures
clear all; close all; clc;

%% Define the feasible set boundaries
x1_min = 0;
x1_max = 4;
x2_min = 0;
x2_max = 3;

% Define the vertices of the rectangle for feasible set M
x1_vertices = [x1_min, x1_max, x1_max, x1_min, x1_min];
x2_vertices = [x2_min, x2_min, x2_max, x2_max, x2_min];

%% Compute the signed distance function l(x) and plot as a 3D surface plot

% Create a grid of points over a specified range
[X1_l, X2_l] = meshgrid(linspace(-2, 6, 200), linspace(-2, 5, 200));

% Initialize the signed distance function s_M(z)
sM_l = zeros(size(X1_l));

% Compute s_M(z) at each grid point
for i = 1:size(X1_l, 1)
    for j = 1:size(X1_l, 2)
        z1 = X1_l(i, j);
        z2 = X2_l(i, j);

        % Check if the point is inside the rectangle
        inside_x = (z1 >= x1_min) && (z1 <= x1_max);
        inside_y = (z2 >= x2_min) && (z2 <= x2_max);

        if inside_x && inside_y
            % Point is inside the rectangle
            d_left   = z1 - x1_min;
            d_right  = x1_max - z1;
            d_bottom = z2 - x2_min;
            d_top    = x2_max - z2;
            d_min    = min([d_left, d_right, d_bottom, d_top]);
            sM_l(i, j) = -d_min;
        else
            % Point is outside the rectangle
            dx = max([x1_min - z1, 0, z1 - x1_max]);
            dy = max([x2_min - z2, 0, z2 - x2_max]);
            sM_l(i, j) = sqrt(dx^2 + dy^2);
        end
    end
end

% Compute l = -s_M(z)
l_values = -sM_l;

% Plot l as a 3D surface plot
figure;

% Use surf to create a 3D surface plot
surf(X1_l, X2_l, l_values, 'EdgeColor', 'none');

% Add labels and title
xlabel('x1');
ylabel('x2');
zlabel('l = -s_M(z)');
title('3D Surface Plot of Signed Distance Function l(x)');

% Adjust the view angle for better visualization
view(45, 30); % Adjust the azimuth and elevation angles

% Add lighting and shading effects for better visualization
shading interp; % Interpolate colors across lines and faces
camlight left;  % Add a light source
lighting phong; % Use Phong lighting model

% Adjust the axes limits
xlim([-2, 6]);
ylim([-2, 5]);
grid on;
hold on;

%% Simulate a trajectory inside the feasible set and overlay on the 3D l(x) plot

% Define initial state for trajectory (inside the feasible set)
x0_traj = [2; 1.5]; % Ensure it's within the feasible set

% Time span for simulation
T_traj = 0.5; % Simulation time for the trajectory
time_span_traj = [0 T_traj];

% Solver options for higher precision
options = odeset('RelTol',1e-8,'AbsTol',1e-10);

% Solve the ODE for the trajectory
[t_traj, x_traj] = ode45(@dynamics, time_span_traj, x0_traj, options);

% Compute l(x(t)) along the trajectory
l_traj_values = arrayfun(@(idx) compute_l(x_traj(idx, :)'), 1:length(t_traj));

% Overlay the trajectory on the 3D l(x) surface plot
plot3(x_traj(:,1), x_traj(:,2), l_traj_values, 'k', 'LineWidth', 2);

% Mark the initial point on the 3D plot
plot3(x0_traj(1), x0_traj(2), l_traj_values(1), 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 8);

% Add legend
legend('Signed Distance Function l(x)', 'Trajectory', 'Initial State', 'Location', 'best');

hold off;

%% Compute the value function V(x) over the adjusted grid

% Define the grid of initial states over the specified range for V(x)
[x1_V, x2_V] = meshgrid(linspace(-2, 6, 40), linspace(-2, 5, 40));

% Time horizon for simulation
T_V = 5; % Adjusted to match the trajectory simulation time
time_span_V = [0 T_V];

% Initialize V(x) matrix
V_values = zeros(size(x1_V));

% Solver options for higher precision
options = odeset('RelTol',1e-8,'AbsTol',1e-10);

% Loop over each initial state
for i = 1:numel(x1_V)
    % Initial state
    x0_V = [x1_V(i); x2_V(i)];

    % Solve the ODE
    [t_V, x_V] = ode45(@dynamics, time_span_V, x0_V, options);

    % Compute l(x(t)) at each time step
    l_traj_V = arrayfun(@(idx) compute_l(x_V(idx, :)'), 1:length(t_V));

    % Compute V(x) as the minimum l over time
    V_values(i) = min(l_traj_V);
end

% Reshape V to match the grid
V_grid = reshape(V_values, size(x1_V));

%% Plot the value function V(x) as a 3D surface plot with trajectory overlay

% Plot V(x) as a 3D surface plot
figure;

% Use surf to create a 3D surface plot
surf(x1_V, x2_V, V_grid, 'EdgeColor', 'none');

% Add labels and title
xlabel('x1');
ylabel('x2');
zlabel('V(x)');
title('3D Surface Plot of Value Function V(x)');

% Adjust the view angle for better visualization
view(45, 30); % Adjust the azimuth and elevation angles

% Add lighting and shading effects for better visualization
shading interp; % Interpolate colors across lines and faces
camlight left;  % Add a light source
lighting phong; % Use Phong lighting model

% Adjust axes
axis equal;
xlim([-2, 6]);
ylim([-2, 5]);
grid on;
hold on;

% Compute V(x(t)) along the trajectory
V_traj_values = arrayfun(@(idx) min(l_traj_values(1:idx)), 1:length(t_traj));

% Overlay the trajectory on the 3D V(x) surface plot
plot3(x_traj(:,1), x_traj(:,2), V_traj_values, 'k', 'LineWidth', 2);

% Mark the initial point on the 3D plot
plot3(x0_traj(1), x0_traj(2), V_traj_values(end), 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 8);

% Add legend
legend('Value Function V(x)', 'Trajectory', 'Initial State', 'Location', 'best');

hold off;

%% Verify the Trajectory

% Display the minimum l(x(t)) along the trajectory
min_l_traj = min(l_traj_values);
disp(['Minimum l(x(t)) along trajectory: ', num2str(min_l_traj)]);

% Check if l(x(t)) >= 0 for all t
if all(l_traj_values > 0)
    disp('The trajectory remains strictly within the feasible set for all time.');
else
    disp('The trajectory touches or exits the feasible set at some point.');
end

%% Plot l(x(t)) vs Time

% Plot l(x(t)) over time
figure;
plot(t_traj, l_traj_values, 'LineWidth', 2);
xlabel('Time t');
ylabel('l(x(t))');
title('Negative Signed Distance Function l(x(t)) Along Trajectory');
grid on;



figure 
plot(V_traj_values)

%% Auxiliary Functions

% Dynamics Function
function dxdt = dynamics(~, x)
    % Dynamics function for ODE solver
    % x is a 2-element vector [x1; x2]

    % System matrices
    A = [0, 1; -1, 0];

    % Disturbance as a function of state
    d = sin(x(1));

    % Disturbance effect
    h = [0; d];

    % Input u = 0
    % u = 0; (Not used since u = 0)

    % Compute the derivative
    dxdt = A * x + h;
end


% Helper Function to Compute l(x)
function l_value = compute_l(x)
    % Computes the negative signed distance function l(x)
    % x is a 2-element vector [x1; x2]

    x1 = x(1);
    x2 = x(2);

    % Feasible set boundaries
    x1_min = 0;
    x1_max = 4;
    x2_min = 0;
    x2_max = 3;

    % Check if the point is inside the rectangle
    inside_x = (x1 >= x1_min) && (x1 <= x1_max);
    inside_y = (x2 >= x2_min) && (x2 <= x2_max);

    if inside_x && inside_y
        % Point is inside the rectangle
        d_left   = x1 - x1_min;
        d_right  = x1_max - x1;
        d_bottom = x2 - x2_min;
        d_top    = x2_max - x2;
        d_min    = min([d_left, d_right, d_bottom, d_top]);
        sM = -d_min;
    else
        % Point is outside the rectangle
        dx = max([x1_min - x1, 0, x1 - x1_max]);
        dy = max([x2_min - x2, 0, x2 - x2_max]);
        sM = sqrt(dx^2 + dy^2);
    end

    % Negative signed distance function
    l_value = -sM;
end
