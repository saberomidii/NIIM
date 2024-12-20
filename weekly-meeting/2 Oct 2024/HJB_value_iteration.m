clear all
clc
close all

addpath(genpath('C:\Users\saber\OneDrive\Desktop\Research\Pablo-Marek-Begum\Weekly Meeting\29 July 2024\ToolboxLS'));
addpath(genpath('C:\Users\saber\OneDrive\Desktop\Research\Pablo-Marek-Begum\Weekly Meeting\29 July 2024\helperOC-master'));

grid_min = [-5; -5; -pi];
grid_max = [5; 5; pi];
N = [100; 100; 31];
g = createGrid(grid_min, grid_max, N, 3); % 3rd dimension is periodic


%%% Define the feasible set 
x_min = 0; % Minimum x-coordinate of the rectangle
x_max = 2;  % Maximum x-coordinate of the rectangle


% Define the desired trajectory and offset functions
desired_trajectory = @(x) 0.75 * x.^2; % Scaled quadratic trajectory
offset_positive = @(x) desired_trajectory(x) + 0.2;
offset_negative = @(x) desired_trajectory(x) - 0.2;


% Create a matrix to store whether each point is within the feasible set
in_feasible_set = zeros(N(1), N(2), N(3));
X = 0;
Y = 0;
Theta = 0;

% Iterate through each point in the grid and check if it is within the feasible set
for i = 1:N(1)
    for j = 1:N(2)
        for k = 1:N(3)
            x = g.xs{1}(i, j, k);
            y = g.xs{2}(i, j, k);
            theta = g.xs{3}(i, j, k);
            
            % Compute y bounds based on the desired trajectory
            if x >= x_min && x <= x_max
                y_min = offset_negative(x);
                y_max = offset_positive(x);
            else
                y_min = -inf; % No bounds if x is outside [x_min, x_max]
                y_max = inf;
            end
            
            % Check if the point (x, y, theta) is within the feasible set
            if (x >= x_min && x <= x_max) && (y >= y_min && y <= y_max)
                in_feasible_set(i, j, k) = -min(vecnorm([x; y] - [X; Y]));
            else
                in_feasible_set(i, j, k) = min(vecnorm([x; y] - [X; Y]));
            end
        end
    end
end

l = -in_feasible_set;
data0 = l; % Use l as the initial value function directly


 %% First subplot - 3D view of the level set
     figure
    subplot(1, 2, 1);
    % Visualize the level set at the current time step
    visualizeLevelSet(g, data0(:,:,:), 'surface', 0, ...
        ['Value function, t = 0 s (3D view)']);
    
    % Set the axis limits
    axis(g.axis);
    xlabel('X');
    ylabel('Y');
    zlabel('\theta');
    grid on;
    
    % Add lighting
    camlight left;
    camlight right;
    lighting phong;
    
    %% Second subplot - Zero level set projected onto (x, y) plane
    subplot(1, 2, 2);
    
    % Compute the minimum of the value function over theta at each (x, y)
    V_xy = min(data0(:,:,:), [], 3);
    
    % Plot the zero level set in (x, y) plane
    contourf(g.xs{1}(:,:,end), g.xs{2}(:,:,end), V_xy', [0 0], 'LineColor', 'r', 'LineWidth', 2);
    hold on;
    % Optionally, plot additional contours
    % contour(g.xs{1}(:,:,1), g.xs{2}(:,:,1), V_xy', 20);
    
    % Set axis limits
    axis([g.min(1) g.max(1) g.min(2) g.max(2)]);
    xlabel('X');
    ylabel('Y');
    title(['Zero Level Set at t = 0  s (Projected onto X-Y plane)']);
    grid on;
    axis equal;

% data0 = shapeCylinder(g, 3, [0; 0; 0], 0.5); % Target at origin with radius 0.5
targets = data0;
extraArgs.targets = targets;

extraArgs.visualize = true;
Q=[0, 0, 0;0 0 0;0 0 0];
R=0;
t0 = 0;
tMax = 1;
dt = 0.01;
tau = t0:dt:tMax;

% targets = zeros([size(data0) length(tau)]);

v=0.2;
omega_max=1;

dCar = DubinsCar([0, 0, 0], omega_max, v);

schemeData.grid = g;
schemeData.dynSys = dCar;
schemeData.uMode = 'max';
schemeData.runningCost = @(x, u) x' * Q * x + u' * R * u;
extraArgs.targets = targets;

[data, tau] = HJIPDE_solve(data0, tau, schemeData, 'none',extraArgs);


% Dubins Car HJB Solver with Quadratic Cost
% This script computes the value function V(x, y, theta) for the Dubins car
% using value iteration to solve the HJB equation with a quadratic cost.

% % Visualization loop
% figure;
% for i = 1:length(tau)
%     % Clear the current figure
% 
% 
%     %% First subplot - 3D view of the level set
%     subplot(1, 2, 1);
%     % Visualize the level set at the current time step
%     visualizeLevelSet(g, data(:,:,:,i), 'surface', 0, ...
%         ['Value function, t = ' num2str(tau(i), '%.2f') ' s (3D view)']);
% 
%     % Set the axis limits
%     axis(g.axis);
%     xlabel('X');
%     ylabel('Y');
%     zlabel('\theta');
%     grid on;
% 
%     % Add lighting
%     camlight left;
%     camlight right;
%     lighting phong;
% 
%     %% Second subplot - Zero level set projected onto (x, y) plane
%     subplot(1, 2, 2);
% 
%     % Compute the minimum of the value function over theta at each (x, y)
%     V_xy = min(data(:,:,:,i), [], 3);
% 
%     % Plot the zero level set in (x, y) plane
%     contourf(g.xs{1}(:,:,1), g.xs{2}(:,:,1), V_xy', [0 0], 'LineColor', 'r', 'LineWidth', 2);
%     hold on;
%     % Optionally, plot additional contours
%     % contour(g.xs{1}(:,:,1), g.xs{2}(:,:,1), V_xy', 20);
% 
%     % Set axis limits
%     axis([g.min(1) g.max(1) g.min(2) g.max(2)]);
%     xlabel('X');
%     ylabel('Y');
%     title(['Zero Level Set at t = ' num2str(tau(i), '%.2f') ' s (Projected onto X-Y plane)']);
%     grid on;
%     axis equal;
% 
%     % Update the figure
%     drawnow;
% 
%     % Pause to control the speed of the animation
%     pause(0.001);  % Adjust the pause duration as needed
% end
% 


% clear; clc; close all;
% 
% %% Parameters
% 
% % Dubins car dynamics parameters
% v = 1.0;              % Constant speed
% omega_max = 1.0;      % Maximum turn rate
% 
% % Cost function weights (Q and R matrices)
% Qx = 1.0;             % Weight for x^2 in state cost
% Qy = 1.0;             % Weight for y^2 in state cost
% Qtheta = 0.0;         % Weight for theta^2 in state cost (set to 0 if not penalizing theta)
% R = 0.1;              % Control cost weight (must be positive)
% 
% % State space bounds
% x_min = -5; x_max = 5; Nx = 51;       % x from -5 to 5 with 51 points
% y_min = -5; y_max = 5; Ny = 51;       % y from -5 to 5 with 51 points
% theta_min = -pi; theta_max = pi; Ntheta = 31; % theta from -pi to pi with 31 points
% 
% % Discretize state space
% x = linspace(x_min, x_max, Nx);       % x grid
% y = linspace(y_min, y_max, Ny);       % y grid
% theta = linspace(theta_min, theta_max, Ntheta); % theta grid
% 
% % Create meshgrid for computation
% [X, Y, Theta] = meshgrid(x, y, theta);
% 
% % Compute grid spacings
% dx = x(2) - x(1);
% dy = y(2) - y(1);
% dtheta = theta(2) - theta(1);
% 
% %% Initialize Value Function
% 
% % Initial guess for V: Quadratic cost based on state variables
% V = Qx * X.^2 + Qy * Y.^2 + Qtheta * Theta.^2;
% 
% % Define the target set (e.g., circle centered at origin with radius r_target)
% r_target = 1;                                      % Radius of target set
% distance = sqrt(X.^2 + Y.^2);                        % Distance from origin
% target_indices = distance <= r_target;               % Logical indices for target set
% 
% % Set value function V to zero at target states
% V(target_indices) = 0;
% 
% %% Value Iteration Parameters
% 
% dt = 0.1;                 % Time step for iteration
% max_iter = 1000;           % Maximum number of iterations
% tolerance = 1e-5;         % Convergence tolerance
% 
% %% Value Iteration Loop
% 
% for iter = 1:max_iter
%     % Compute gradients of V using finite differences
%     [V_x, V_y, V_theta] = gradient(V, dx, dy, dtheta);
% 
%     % Compute optimal control u_star at each point
%     u_star = -V_theta / (2 * R);                        % Optimal control without constraints
%     u_star = max(min(u_star, omega_max), -omega_max);   % Apply control constraints
% 
%     % Compute Hamiltonian H at each point
%     H = Qx * X.^2 + Qy * Y.^2 + Qtheta * Theta.^2 ...   % State cost
%         - (V_theta.^2) / (4 * R) ...                    % Control cost after minimization
%         + V_x .* (v * cos(Theta)) ...                   % Dynamics in x-direction
%         + V_y .* (v * sin(Theta));                      % Dynamics in y-direction
% 
%     % Update value function V
%     V_new = V;
%     V_new(~target_indices) = V(~target_indices) + dt * H(~target_indices);
% 
%     % Ensure V remains zero at target states
%     V_new(target_indices) = 0;
% 
%     % Check convergence
%     delta = max(abs(V_new(~target_indices) - V(~target_indices)));
%     fprintf('Iteration %d, delta = %f\n', iter, delta);
%     if delta < tolerance
%         fprintf('Converged after %d iterations.\n', iter);
%         break;
%     end
% 
%     % Update V for next iteration
%     V = V_new;
% end
% 
% %% Visualization
% 
% % Since V is 3D data (x, y, theta), fix theta to a specific value for visualization
% theta_index = ceil(Ntheta / 2);       % Index corresponding to theta = 0
% V_xy = V(:, :, theta_index);          % Extract V at theta = 0
% 
% % Plot the value function as a surface
% figure;
% surf(x, y, V_xy');                    % Note: Transpose for correct orientation
% xlabel('x');
% ylabel('y');
% zlabel('Value Function V(x, y, \theta=0)');
% title('Value Function for Dubins Car at \theta = 0');
% shading interp;
% colorbar;
% view(30, 45);                         % Adjust view angle
% grid on;
% 
% % Plot the value function as a contour plot
% figure;
% contourf(x, y, V_xy', 20);            % 20 contour levels
% xlabel('x');
% ylabel('y');
% title('Value Function Contour at \theta = 0');
% colorbar;
% axis equal;
% grid on;
% 
% 
% % Install and use vol3d for visualization of volumetric data (vol3d is part of MATLAB File Exchange)
% vol3d('CData', V);
% view(3);
% axis tight;
% colormap('jet');


% HJIPDE_solve_test(whatTest)
%   Tests the HJIPDE_solve function as well as provide an example of how to
%   use it.
%
% whatTest - Argument that can be used to test a particular feature
%     'minWith':   Test the minWith functionality
%     'tvTargets': Test the time-varying targets
%     'singleObs': Test with a single static obstacle
%     'tvObs':     Test with time-varying obstacles
%     'obs_stau':  single obstacle over a few time steps
%     'stopInit':  Test the functionality of stopping reachable set
%                  computation once it includes the initial state
%     'stopSetInclude':
%         Test the functionality of stopping reacahble set computation once it
%         contains some set
%     'stopSetIntersect':
%         Test the functionality of stopping reacahble set computation once it
%         intersects some set
%     'plotData':  Test the functionality of plotting reachable sets as
%                  they are being computed
% 
% whatTest = 'minWith';
% 
% 
% %% Grid
% grid_min = [-5; -5; -pi]; % Lower corner of computation domain
% grid_max = [5; 5; pi];    % Upper corner of computation domain
% N = [41; 41; 41];         % Number of grid points per dimension
% pdDims = 3;               % 3rd dimension is periodic
% g = createGrid(grid_min, grid_max, N, pdDims);
% % Use "g = createGrid(grid_min, grid_max, N);" if there are no periodic
% % state space dimensions
% 
% %% target set
% R = 1;
% data0 = shapeCylinder(g, 3, [0; 0; 0], R);
% % also try shapeRectangleByCorners, shapeSphere, etc.
% 
% %% time vector
% t0 = 0;
% tMax = 2;
% dt = 0.025;
% tau = t0:dt:tMax;
% % If intermediate results are not needed, use tau = [t0 tMax];
% 
% %% problem parameters
% speed = 1;
% wMax = 1;
% 
% %% Pack problem parameters
% % Dynamical system parameters
% dCar = DubinsCar([0, 0, 0], wMax, speed);
% schemeData.grid = g;
% schemeData.dynSys = dCar;
% 
% %% Compute time-dependent value function
% if strcmp(whatTest, 'minWith')
%   minWiths = {'none', 'zero'};
%   % selecting 'zero' computes reachable tube (usually, choose this option)
%   % selecting 'none' computes reachable set
%   % selecting 'target' computes reachable tube, but only use this if there are
%   %   obstacles (constraint/avoid sets) in the state space
% 
%   numPlots = 4;
%   spC = ceil(sqrt(numPlots));
%   spR = ceil(numPlots / spC);
% 
%   for i = 1:length(minWiths)
%     [data, tau, ~] = HJIPDE_solve(data0, tau, schemeData, minWiths{i});
% 
%     % Visualize
%     figure;
%     for j = 1:numPlots
%       subplot(spR, spC, j)
%       ind = ceil(j * length(tau) / numPlots);
%       visualizeLevelSet(g, data(:,:,:,ind), 'surface', 0, ...
%         ['TD value function, t = ' num2str(tau(ind))]);
%       axis(g.axis)
%       camlight left
%       camlight right
%       drawnow
%     end
%   end
% end
% % In practice, most of the time, the above for loop is not needed, and the
% % code below is also not needed. Simply select an minWith option, and then
% % also input obstacles if they are present.
% 
% % Change visualization code as necessary
% 
% %% Test using time-varying targets
% if strcmp(whatTest, 'tvTargets')
%   % Specify targets
%   targets = zeros([size(data0) length(tau)]);
%   for i = 1:length(tau)
%     targets(:,:,:,i) = shapeCylinder(g, 3, [1.5; 1.5; 0], i/length(tau)*R);
%   end
%   extraArgs.targets = targets;
% 
%   [data, tau, ~] = HJIPDE_solve(data0, tau, schemeData, 'none', extraArgs);
% 
%   % Visualize
%   figure;
%   numPlots = 4;
%   spC = ceil(sqrt(numPlots));
%   spR = ceil(numPlots / spC);
%   for i = 1:numPlots
%     subplot(spR, spC, i)
%     ind = ceil(i * length(tau) / numPlots);
%     visualizeLevelSet(g, data(:,:,:,ind), 'surface', 0, ...
%       ['TD value function, t = ' num2str(tau(ind))]);
%     axis(g.axis)
%     camlight left
%     camlight right
%     drawnow
%   end
% end
% 
% %% Test using single obstacle
% if strcmp(whatTest, 'singleObs')
%   obstacles = shapeCylinder(g, 3, [1.5; 1.5; 0], 0.75*R);
%   extraArgs.obstacles = obstacles;
% 
%   targets = data0;
%   extraArgs.targets = targets;
%   extraArgs.visualize = true;
% 
%   numPlots = 4;
%   spC = ceil(sqrt(numPlots));
%   spR = ceil(numPlots / spC);
% 
%   [data, tau, ~] = HJIPDE_solve(data0, tau, schemeData, 'none', extraArgs);
% 
%   % Visualize
%   figure;
%   for i = 1:numPlots
%     subplot(spR, spC, i)
%     ind = ceil(i * length(tau) / numPlots);
%     visualizeLevelSet(g, data(:,:,:,ind), 'surface', 0, ...
%       ['TD value function, t = ' num2str(tau(ind))]);
%     axis(g.axis)
%     camlight left
%     camlight right
%     drawnow
%   end
% end
% 
% %% Test using time-varying obstacle
% if strcmp(whatTest, 'tvObs')
%   obstacles = zeros([size(data0) length(tau)]);
%   for i = 1:length(tau)
%     obstacles(:,:,:,i) = shapeCylinder(g, 3, [1.5; 1.5; 0], i/length(tau)*R);
%   end
%   extraArgs.obstacles = obstacles;
% 
%   targets = data0;
%   extraArgs.targets = targets;
% 
%   extraArgs.visualize = true;
% 
%   numPlots = 4;
%   spC = ceil(sqrt(numPlots));
%   spR = ceil(numPlots / spC);
% 
%   [data, tau, ~] = HJIPDE_solve(data0, tau, schemeData, 'none', extraArgs);
% 
%   % Visualize
%   figure;
%   for i = 1:numPlots
%     subplot(spR, spC, i)
%     ind = ceil(i * length(tau) / numPlots);
%     visualizeLevelSet(g, data(:,:,:,ind), 'surface', 0, ...
%       ['TD value function, t = ' num2str(tau(ind))]);
%     axis(g.axis)
%     camlight left
%     camlight right
%     drawnow
%   end
% end
% 
% %% Test using single obstacle but few time steps
% if strcmp(whatTest, 'obs_stau')
%   obstacles = shapeCylinder(g, 3, [1.5; 1.5; 0], 0.75*R);
%   tau = linspace(0, 2, 5);
%   extraArgs.obstacles = obstacles;
% 
%   targets = data0;
%   extraArgs.targets = targets;
% 
%   numPlots = 4;
%   spC = ceil(sqrt(numPlots));
%   spR = ceil(numPlots / spC);
% 
%   [data, tau] = HJIPDE_solve(data0, tau, schemeData, 'none', extraArgs);
% 
%   % Visualize
%   figure;
%   for i = 1:numPlots
%     subplot(spR, spC, i)
%     ind = ceil(i * length(tau) / numPlots);
%     visualizeLevelSet(g, data(:,:,:,ind), 'surface', 0, ...
%       ['TD value function, t = ' num2str(tau(ind))]);
%     axis(g.axis)
%     camlight left
%     camlight right
%     drawnow
%   end
% end
% 
% %% Test the inclusion of initial state
% if strcmp(whatTest, 'stopInit')
%   extraArgs.stopInit = [-1.1, -1.1, 0];
%   tau = linspace(0, 2, 50);
% 
%   numPlots = 4;
%   spC = ceil(sqrt(numPlots));
%   spR = ceil(numPlots / spC);
% 
%   extraArgs.visualize = true;
%   extraArgs.deleteLastPlot = true;
%   extraArgs.plotData.plotDims = [1 1 0];
%   extraArgs.plotData.projpt = extraArgs.stopInit(3);
%   [data, tau, extraOuts] = ...
%     HJIPDE_solve(data0, tau, schemeData, 'none', extraArgs);
% 
%   % Visualize
%   figure;
%   for i = 1:numPlots
%     subplot(spR, spC, i)
%     ind = ceil(i * length(tau) / numPlots);
%     h = visSetIm(g, data(:,:,:,ind));
%     h.FaceAlpha = 0.5;
%     axis(g.axis)
%     title(['TD value function, t = ' num2str(tau(ind))]);
% 
%     hold on
%     plot3(extraArgs.stopInit(1), ...
%       extraArgs.stopInit(2), extraArgs.stopInit(3), '*')
% 
%     camlight left
%     camlight right
%     drawnow
%   end
% 
%   val = eval_u(g, data(:,:,:,end), extraArgs.stopInit);
%   fprintf('Value at initial condition is %f\n', val)
% end
% 
% %% Test the inclusion of some set
% if strcmp(whatTest, 'stopSetInclude')
%   extraArgs.stopSetInclude = shapeSphere(g, [-1.1 1.1 0], 0.5);
%   tau = linspace(0, 2, 5);
% 
%   numPlots = 4;
%   spC = ceil(sqrt(numPlots));
%   spR = ceil(numPlots / spC);
% 
%   [data, tau] = HJIPDE_solve(data0, tau, schemeData, 'none', extraArgs);
% 
%   % Visualize
%   figure;
%   for i = 1:numPlots
%     subplot(spR, spC, i)
%     ind = ceil(i * length(tau) / numPlots);
%     visSetIm(g, extraArgs.stopSetInclude, 'b');
%     h = visualizeLevelSet(g, data(:,:,:,ind), 'surface', 0, ...
%       ['TD value function, t = ' num2str(tau(ind))]);
%     h.FaceAlpha = 0.6;
%     axis(g.axis)
%     camlight left
%     camlight right
%     drawnow
%   end
% end
% 
% %% Test intersection of some set
% if strcmp(whatTest, 'stopSetIntersect')
%   extraArgs.stopSetIntersect = shapeSphere(g, [-1.25 1.25 0], 0.5);
%   tau = linspace(0, 1, 11);
% 
%   numPlots = 4;
%   spC = ceil(sqrt(numPlots));
%   spR = ceil(numPlots / spC);
% 
%   [data, tau] = HJIPDE_solve(data0, tau, schemeData, 'none', extraArgs);
% 
%   % Visualize
%   figure;
%   for i = 1:numPlots
%     subplot(spR, spC, i)
%     ind = ceil(i * length(tau) / numPlots);
%     visSetIm(g, extraArgs.stopSetIntersect, 'b');
%     h = visualizeLevelSet(g, data(:,:,:,ind), 'surface', 0, ...
%       ['TD value function, t = ' num2str(tau(ind))]);
%     h.FaceAlpha = 0.6;
%     axis(g.axis)
%     camlight left
%     camlight right
%     drawnow
%   end
% end
% 
% %% Test the intermediate plotting
% if strcmp(whatTest, 'plotData')
%   tau = linspace(0, 2, 51);
% 
%   extraArgs.visualize = true;
%   extraArgs.plotData.plotDims = [1, 1, 0];
%   extraArgs.plotData.projpt = -3*pi/4;
%   extraArgs.deleteLastPlot = true;
% 
%   % Moving obstacles
%   obstacles = zeros([size(data0) length(tau)]);
%   for i = 1:length(tau)
%     obstacles(:,:,:,i) = shapeCylinder(g, 3, [1.5; 1.5; 0], i/length(tau)*R);
%   end
%   extraArgs.obstacles = obstacles;
% 
%   numPlots = 4;
%   spC = ceil(sqrt(numPlots));
%   spR = ceil(numPlots / spC);
% 
%   [data, tau, extraOuts] = HJIPDE_solve(data0, tau, schemeData, 'none', ...
%     extraArgs);
% 
%   % Visualize
%   figure;
%   for i = 1:numPlots
%     subplot(spR, spC, i)
%     ind = ceil(i * length(tau) / numPlots);
%     visualizeLevelSet(g, data(:,:,:,ind), 'surface', 0, ...
%       ['TD value function, t = ' num2str(tau(ind))]);
%     axis(g.axis)
%     camlight left
%     camlight right
%     drawnow
%   end
% end
% 
% %% Test starting from saved data (where data0 has dimension g.dim + 1)
% if strcmp(whatTest, 'savedData')
%   % Compute data1
%   extraArgs.visualize = true;
%   data1 = HJIPDE_solve(data0, tau, schemeData, 'zero', extraArgs);
% 
%   % Cut off data1 at tcutoff
%   tcutoff = 0.5;
%   dataSaved = data1;
%   dataSaved(:, :, :, tau>tcutoff) = 0;
%   extraArgs.istart = nnz(tau<=tcutoff) + 1;
% 
%   % Continue computing
%   data2 = HJIPDE_solve(dataSaved, tau, schemeData, 'zero', extraArgs);
% 
%   % Plot the two results and compare
%   figure;
%   h1 = visSetIm(g, data1(:,:,:,end));
%   h1.FaceAlpha = 0.5;
%   hold on
%   h2 = visSetIm(g, data2(:,:,:,end), 'b');
%   h2.FaceAlpha = 0.5;
% 
%   % Display error
%   disp(['Computation from saved data differs from full computation by ' ...
%     'an error of ' num2str(sum((data1(:) - data2(:)).^2))])
% end
% 
% if strcmp(whatTest, 'stopConverge')
%   % Parameters
%   N = 61*ones(3,1);
%   grid_min = [-25; -20; 0];
%   grid_max = [25; 20; 2*pi];
%   pdDims = 3;
% 
%   va = 5;
%   vb = 5;
%   uMax = 1;
%   dMax = 1;
% 
%   captureRadius = 5;
% 
%   g = createGrid(grid_min, grid_max, N, pdDims);
%   data0 = shapeCylinder(g, 3, [0;0;0], captureRadius);
%   dynSys = DubinsCarCAvoid([0;0;0], uMax, dMax, va, vb); 
% 
%   tMax = 5;
%   dt = 0.01;
%   tau = 0:dt:tMax;
% 
%   schemeData.grid = g;
%   schemeData.dynSys = dynSys;
%   schemeData.uMode = 'max';
%   schemeData.dMode = 'min';
% 
%   extraArgs.stopConverge = true;
%   extraArgs.convergeThreshold = 1e-3;
%   extraArgs.visualize = true;
%   extraArgs.deleteLastPlot = true;
%   data = HJIPDE_solve(data0, tau, schemeData, 'zero', extraArgs);
% end
% 
% %% Low memory mode
% if strcmp(whatTest, 'low_memory')
%   obstacles = zeros([size(data0) length(tau)]);
%   for i = 1:length(tau)
%     obstacles(:,:,:,i) = shapeCylinder(g, 3, [1.5; 1.5; 0], i/length(tau)*R);
%   end
%   extraArgs.obstacles = obstacles;  
%   extraArgs.quiet = true;
% 
%   tic
%   data_normal = HJIPDE_solve(data0, tau, schemeData, 'zero', extraArgs);
%   fprintf('Normal mode time: %f seconds\n', toc)
% 
%   extraArgs.low_memory = true;
%   tic
%   data_low_mem = HJIPDE_solve(data0, tau, schemeData, 'zero', extraArgs);
%   fprintf('Low memory mode time: %f seconds\n', toc)
% 
%   error = max(abs(data_normal(:) - data_low_mem(:)));
%   fprintf('Error = %f\n', error)
% end
% 
% %% flip outputs in low memory mode
% if strcmp(whatTest, 'flip_output')
%   obstacles = zeros([size(data0) length(tau)]);
%   for i = 1:length(tau)
%     obstacles(:,:,:,i) = shapeCylinder(g, 3, [1.5; 1.5; 0], i/length(tau)*R);
%   end
%   extraArgs.obstacles = obstacles;  
%   extraArgs.quiet = true;
% 
%   tic
%   data_normal = HJIPDE_solve(data0, tau, schemeData, 'zero', extraArgs);
%   fprintf('Normal mode time: %f seconds\n', toc)
% 
%   extraArgs.flip_output = true;
%   extraArgs.low_memory = true;
%   tic
%   data_low_mem = HJIPDE_solve(data0, tau, schemeData, 'zero', extraArgs);
%   fprintf('Low memory mode time: %f seconds\n', toc)
%   data_low_mem = flip(data_low_mem, 4);
% 
%   error = max(abs(data_normal(:) - data_low_mem(:)));
%   fprintf('Error = %f\n', error)
% 
%   figure
%   subplot(1,2,1)
%   visSetIm(g, data_normal);
%   subplot(1,2,2)
%   visSetIm(g, data_low_mem);
% end


% clear all
% close all
% clc
% 
% %% Grid
% grid_min = [-5; -5; -pi]; % Lower corner of computation domain
% grid_max = [5; 5; pi];    % Upper corner of computation domain
% N = [41; 41; 41];         % Number of grid points per dimension
% pdDims = 3;               % 3rd dimension is periodic
% g = createGrid(grid_min, grid_max, N, pdDims);
% % Use "g = createGrid(grid_min, grid_max, N);" if there are no periodic
% % state space dimensions
% 
% %% target set
% R = 1;
% data0 = shapeCylinder(g, 3, [0; 0; 0], R);
% % also try shapeRectangleByCorners, shapeSphere, etc.
% 
% %% time vector
% t0 = 0;
% tMax = 2;
% dt = 0.025;
% tau = t0:dt:tMax;
% % If intermediate results are not needed, use tau = [t0 tMax];
% 
% %% problem parameters
% speed = 1;
% wMax = 1;
% 
% %% Pack problem parameters
% % Dynamical system parameters
% dCar = DubinsCar([0, 0, 0], wMax, speed);
% schemeData.grid = g;
% schemeData.dynSys = dCar;
% 
% %% Compute time-dependent value function
% 
% [data, tau, ~] = HJIPDE_solve(data0, tau, schemeData);

% Clear workspace and close figures
% clear; clc; close all;
% 
% Dubins car parameters
% v = 1.0;                    % Constant speed
% omega_max = 1.0;            % Maximum turn rate
% 
% Cost function weights
% Qx = 1.0;                   % State cost weight for x
% Qy = 1.0;                   % State cost weight for y
% Qtheta = 0.1;               % State cost weight for theta
% R = 0.1;                    % Control cost weight
% 
% State space bounds and discretization
% x_min = -5; x_max = 5; Nx = 51;
% y_min = -5; y_max = 5; Ny = 51;
% theta_min = -pi; theta_max = pi; Ntheta = 31;
% 
% x = linspace(x_min, x_max, Nx);
% y = linspace(y_min, y_max, Ny);
% theta = linspace(theta_min, theta_max, Ntheta);
% 
% Create meshgrid for state variables
% [X, Y, Theta] = meshgrid(x, y, theta);
% 
% Control input discretization
% Nu = 11;
% u = linspace(-omega_max, omega_max, Nu);
% 
% Initialize value function V(x) with infinity
% V = inf(Nx, Ny, Ntheta);
% 
% Define target set (circle at origin with radius r_target)
% r_target = 0.5;
% target_indices = sqrt(X.^2 + Y.^2) <= r_target;
% 
% Set value function to zero at target states
% V(target_indices) = 0;
% 
% Initialize policy array
% policy = zeros(Nx, Ny, Ntheta);
% 
% Value iteration parameters
% max_iterations = 100;
% tolerance = 1e-3;
% delta = Inf;
% iteration = 0;
% gamma = 1.0;                % Discount factor (gamma = 1 for infinite horizon)
% dt = 0.1;                   % Time step for simulation
% 
% State cost matrix Q
% Q = diag([Qx, Qy, Qtheta]);
% 
% Start value iteration
% while delta > tolerance && iteration < max_iterations
%     V_prev = V;
%     delta = 0;
%     iteration = iteration + 1;
%     fprintf('Iteration %d\n', iteration);
% 
%     Loop over all states
%     for ix = 1:Nx
%         for iy = 1:Ny
%             for itheta = 1:Ntheta
%                 Skip target states
%                 if target_indices(ix, iy, itheta)
%                     continue;
%                 end
% 
%                 Current state
%                 x_curr = X(ix, iy, itheta);
%                 y_curr = Y(ix, iy, itheta);
%                 theta_curr = Theta(ix, iy, itheta);
% 
%                 Initialize minimum cost
%                 min_cost = Inf;
%                 best_u = 0;
% 
%                 Loop over control inputs
%                 for iu = 1:Nu
%                     u_curr = u(iu);
% 
%                     Compute next state using dynamics
%                     x_next = x_curr + dt * v * cos(theta_curr);
%                     y_next = y_curr + dt * v * sin(theta_curr);
%                     theta_next = theta_curr + dt * u_curr;
% 
%                     Wrap theta to [-pi, pi]
%                     theta_next = atan2(sin(theta_next), cos(theta_next));
% 
%                     Check if next state is within bounds
%                     if x_next < x_min || x_next > x_max || ...
%                        y_next < y_min || y_next > y_max
%                         continue; % Skip if out of bounds
%                     end
% 
%                     Find nearest grid indices
%                     [~, ix_next] = min(abs(x - x_next));
%                     [~, iy_next] = min(abs(y - y_next));
%                     [~, itheta_next] = min(abs(theta - theta_next));
% 
%                     Compute running cost
%                     state = [x_curr; y_curr; theta_curr];
%                     running_cost = state' * Q * state + R * u_curr^2;
% 
%                     Total cost-to-go
%                     total_cost = running_cost * dt + gamma * V_prev(ix_next, iy_next, itheta_next);
% 
%                     Update minimum cost
%                     if total_cost < min_cost
%                         min_cost = total_cost;
%                         best_u = u_curr;
%                     end
%                 end
% 
%                 Update value function and policy
%                 V(ix, iy, itheta) = min_cost;
%                 policy(ix, iy, itheta) = best_u;
% 
%                 Update delta
%                 delta = max(delta, abs(V(ix, iy, itheta) - V_prev(ix, iy, itheta)));
%             end
%         end
%     end
% 
%     fprintf('Max change in V: %f\n', delta);
% end
% 
% fprintf('Value iteration converged after %d iterations.\n', iteration);
% 
% Find index for theta = 0
% [~, theta_zero_idx] = min(abs(theta));
% 
% Extract value function at theta = 0
% V_xy = V(:, :, theta_zero_idx);
% 
% Plot value function
% figure;
% surf(x, y, V_xy');
% xlabel('x');
% ylabel('y');
% zlabel('V(x, y, \theta=0)');
% title('Value Function at \theta = 0');
% shading interp;
% colorbar;
% view(30, 45);
% grid on;
% 
% Extract policy at theta = 0
% policy_xy = policy(:, :, theta_zero_idx);
% 
% Plot policy using quiver
% figure;
% [X_plot, Y_plot] = meshgrid(x, y);
% u_plot = policy_xy';
% v_plot = zeros(size(u_plot));
% 
% quiver(X_plot, Y_plot, cos(u_plot), sin(u_plot));
% xlabel('x');
% ylabel('y');
% title('Optimal Control Policy at \theta = 0');
% grid on;
% clear 
% clc
% close all
% 
% 
% whatTest = 'minWith';
% %% Grid
% grid_min = [-5; -5; -pi]; % Lower corner of computation domain
% grid_max = [5; 5; pi];    % Upper corner of computation domain
% N = [41; 41; 41];         % Number of grid points per dimension
% pdDims = 3;               % 3rd dimension is periodic
% g = createGrid(grid_min, grid_max, N, pdDims);
% % Use "g = createGrid(grid_min, grid_max, N);" if there are no periodic
% % state space dimensions
% 
% %% target set
% R = 1;
% data0 = shapeCylinder(g, 3, [0; 0; 0], R);
% % also try shapeRectangleByCorners, shapeSphere, etc.
% 
% %% time vector
% t0 = 0;
% tMax = 2;
% dt = 0.025;
% tau = t0:dt:tMax;
% % If intermediate results are not needed, use tau = [t0 tMax];
% 
% %% problem parameters
% speed = 1;
% wMax = 1;
% 
% %% Pack problem parameters
% % Dynamical system parameters
% dCar = DubinsCar([0, 0, 0], wMax, speed);
% schemeData.grid = g;
% schemeData.dynSys = dCar;
% 
% %% Compute time-dependent value function
% if strcmp(whatTest, 'minWith')
%   minWiths = {'none', 'zero'};
%   % selecting 'zero' computes reachable tube (usually, choose this option)
%   % selecting 'none' computes reachable set
%   % selecting 'target' computes reachable tube, but only use this if there are
%   %   obstacles (constraint/avoid sets) in the state space
% 
%   numPlots = 4;
%   spC = ceil(sqrt(numPlots));
%   spR = ceil(numPlots / spC);
% 
%   for i = 1:length(minWiths)
%     [data, tau, ~] = HJIPDE_solve(data0, tau, schemeData, minWiths{i});
% 
%     % Visualize
%     figure;
%     for j = 1:numPlots
%       subplot(spR, spC, j)
%       ind = ceil(j * length(tau) / numPlots);
%       visualizeLevelSet(g, data(:,:,:,ind), 'surface', 0, ...
%         ['TD value function, t = ' num2str(tau(ind))]);
%       axis(g.axis)
%       camlight left
%       camlight right
%       drawnow
%     end
%   end
% end


%%Lax–Friedrichs sweeping scheme for static
% Hamilton–Jacobi equations q

