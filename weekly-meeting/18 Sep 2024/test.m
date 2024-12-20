% Clear the workspace and close all figures
clear all; close all; clc;

% Add paths to helperOC and ToolboxLS toolboxes
addpath(genpath('path_to_helperOC'));    % Replace with your actual path
addpath(genpath('path_to_ToolboxLS'));   % Replace with your actual path

%% Grid Setup
% Define the grid for state variables: x, y, theta
grid_min = [-5; -5; -pi];   % Lower bounds [x; y; theta]
grid_max = [5; 5; pi];      % Upper bounds [x; y; theta]
N = [101; 101; 51];         % Number of grid points per dimension

g = createGrid(grid_min, grid_max, N);

%% Define the Safe Set X
% For this example, let's define X as a circular region centered at the origin with radius 4
center = [0; 0];
radius = 4;

% Compute the signed distance function for the safe set X
dataX = shapeCylinder(g, 3, [center; 0], radius);   % Cylindrical shape along theta axis

%% Dubins Car Dynamics
% Parameters for the Dubins car
speed = 1;          % Constant forward speed
wMax = 1;           % Maximum steering rate (u ∈ [-wMax, wMax])

% Create the Dubins car dynamic system
dCar = DubinsCar([], wMax, speed);

%% Time Vector
t0 = 0;             % Start time
tMax = 10;          % End time (should be large enough to approximate infinite horizon)
dt = 0.1;           % Time step
tau = t0:dt:tMax;   % Time vector

%% Scheme Data
schemeData.grid = g;
schemeData.dynSys = dCar;
schemeData.accuracy = 'medium';   % Can be 'low', 'medium', or 'high'
schemeData.uMode = 'max';         % Control aims to maximize the value function

%% Solve the HJ PDE to Compute the Value Function V(x)
% The initial value function is the signed distance function of the safe set X
data0 = dataX;

% Set extra arguments for HJI PDE solver
HJIextraArgs.visualize = true;                % Enable visualization
HJIextraArgs.fig_num = 1;                     % Figure number
HJIextraArgs.deleteLastPlot = true;           % Delete last plot in animation
HJIextraArgs.plotData.plotDims = [1 1 0];     % Plot x and y dimensions
HJIextraArgs.plotData.projpt = 0;             % Fix theta at 0 for plotting

% Solve the HJ PDE
[data, tau2] = HJIPDE_solve(data0, tau, schemeData, 'zero', HJIextraArgs);

%% Extract the Value Function V at Final Time
V = data(:, :, :, end);

%% Compute the Optimal Safe Control Policy u_V(x)
% Compute the spatial gradients of V
[gradV] = computeGradients(g, V);

% Initialize the optimal control input array
u_opt = zeros(size(V));

% Compute the optimal control input at each grid point
for idx = 1:numel(V)
    % Extract the gradient at the current grid point
    grad_Vtheta = gradV{3}(idx);
    
    % Compute the optimal control input based on the control mode
    if grad_Vtheta >= 0
        u_opt(idx) = wMax;   % u = wMax
    else
        u_opt(idx) = -wMax;  % u = -wMax
    end
end

% Reshape u_opt to match the grid dimensions
u_opt = reshape(u_opt, size(V));

%% Visualize the Invariant Set and Optimal Control Policy at theta = 0
% Find the index where theta is closest to 0
[~, theta_index] = min(abs(g.vs{3}));

% Extract slices at theta = 0
V_theta0 = V(:, :, theta_index);
u_opt_theta0 = u_opt(:, :, theta_index);

% Plot the value function V
figure;
contourf(g.xs{1}(:, :, theta_index), g.xs{2}(:, :, theta_index), V_theta0, 50);
colorbar;
xlabel('x');
ylabel('y');
title('Value Function V at \theta = 0');
axis equal;
grid on;

% Overlay the zero level set (boundary of the invariant set)
hold on;
contour(g.xs{1}(:, :, theta_index), g.xs{2}(:, :, theta_index), V_theta0, [0 0], 'k', 'LineWidth', 2);
hold off;

% Plot the optimal control input u_opt
figure;
imagesc(g.vs{1}, g.vs{2}, u_opt_theta0');
set(gca, 'YDir', 'normal');
colorbar;
xlabel('x');
ylabel('y');
title('Optimal Control Input u^*_V(x) at \theta = 0');
axis equal;
