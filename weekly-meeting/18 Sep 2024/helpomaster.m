% Clear the workspace and close all figures
clear all; close all; clc;

% Add paths to helperOC-master and Level Set Methods toolbox
addpath(genpath('C:\Users\saber\OneDrive\Desktop\Research\NIIM\weekly-meeting\29 July 2024\helperOC-master'));
addpath(genpath('C:\Users\saber\OneDrive\Desktop\Research\NIIM\weekly-meeting\29 July 2024\ToolboxLS'));

%% Grid setup
grid_min = [-5; -5; -pi];   % Lower bounds [x; y; theta]
grid_max = [5; 5; pi];      % Upper bounds [x; y; theta]
N = [100; 100; 50];         % Number of grid points in each dimension

g = createGrid(grid_min, grid_max, N);

%%% Define the feasible set 
x_min = 0; % Minimum x-coordinate of the rectangle
x_max = 2;  % Maximum x-coordinate of the rectangle
theta_min = -0.5; % Minimum theta
theta_max = 0.5;  % Maximum theta

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
            if (x >= x_min && x <= x_max) && (y >= y_min && y <= y_max) && (theta >= theta_min && theta <= theta_max)
                in_feasible_set(i, j, k) = -min(vecnorm([x; y; theta] - [X; Y; Theta]));
            else
                in_feasible_set(i, j, k) = min(vecnorm([x; y; theta] - [X; Y; Theta]));
            end
        end
    end
end

l = -in_feasible_set;
data0 = l; % Use l as the initial value function directly

% Dubins car dynamics
speed = 0.2;
wMax = 1;
dCar = DubinsCar([], wMax, speed);

%% Time vector
t0 = 0;
tMax = 19;
dt = 0.1;
tau = t0:dt:tMax;

%% Scheme data
schemeData.grid = g;
schemeData.dynSys = dCar;
schemeData.accuracy = 'high'; % Options: 'low', 'medium', 'high'
schemeData.uMode = 'max';    % Minimize the cost function

%% Solve the HJB PDE with target function
HJIextraArgs.visualize = true;                % Enable visualization
HJIextraArgs.fig_num = 1;                     % Figure number
HJIextraArgs.deleteLastPlot = true;           % Delete last plot in animation
HJIextraArgs.targetFunction = -l;              % Target function

HJIextraArgs.makeVideo = true;                % Enable video saving
HJIextraArgs.videoFilename = 'HJIPDE_Video.mp4'; % Specify the video filename
HJIextraArgs.frameRate = 10;                  % Set the frame rate


[V, Time_vector] = HJIPDE_solve(data0, tau, schemeData, 'minVWithTarget', HJIextraArgs);

V=-V;

dV_dtheta = gradient(V, g.dx(3));
wRange = [-1 1];



uOpt = (dV_dtheta >= 0) * wRange(2) + (dV_dtheta < 0) * wRange(1);

% original_time = linspace(1, 1900, 101);

% Define the time points for your closed-loop system (1 to 1900)
% target_time = 1:1900;

% Initialize the interpolated optimal control matrix
% uOpt_interpolated = zeros(100, 100, 50, 1900);

% % Interpolate uOpt along the time dimension
% for i = 1:100
%     for j = 1:100
%         for k = 1:50
%             % Extract the uOpt values at different times for each (x, y, theta) point
%             uOpt_values = squeeze(uOpt(i, j, k, :));
%             % Interpolate these values to match the target_time
%             uOpt_interpolated(i, j, k, :) = interp1(original_time, uOpt_values, target_time, 'linear');
%         end
%     end
% end

save("uOpt.mat",'uOpt')
save('g.mat', 'g');
save('V.mat', 'V');


% 
% 
% V_final = V(:, :, :, end);
% 
% % Define slices for different theta values
% 
% % Define slices for different theta values
% theta_slices = [-0.5, 0, 0.5]; % Values of theta at which to take slices
% 
% % Create a figure to hold multiple subplots
% figure;
% num_slices = length(theta_slices);
% for idx = 1:num_slices
%     % Find the closest index in the theta grid to the desired slice value
%     [~, theta_idx] = min(abs(g.vs{3} - theta_slices(idx)));
% 
%     % Extract the slice of V_final at the given theta index
%     V_slice = V_final(:, :, theta_idx);
% 
%     % Create a subplot for each slice
%     subplot(1, num_slices, idx);
%     contour(g.xs{1}(:,:,theta_idx), g.xs{2}(:,:,theta_idx), V_slice, [0 0], 'LineWidth', 2);
%     xlabel('x');
%     ylabel('y');
%     title(['Zero Level Set at \theta = ', num2str(theta_slices(idx))]);
%     axis equal;
% 
%     % Set smaller x and y range for zooming in
%     xlim([0, 0.5]); % Adjust these limits as needed
%     ylim([-0.2, 0.4]); % Adjust these limits as needed
% end
% 
% % Add a super title for the figure
% sgtitle('Zero Level Set of the Value Function at Different \theta Values');
% 
% 
% % Compute the spatial gradients of V_final
% [gradV_x, gradV_y, gradV_theta] = gradient(V_final, g.dx(1), g.dx(2), g.dx(3));
% 
% % Define the state at which you want to evaluate V_final
% state = [0; 0; 0.2];
% 
% % Use interpn to interpolate V_final at the desired state
% value_at_state = interpn(g.xs{1}, g.xs{2}, g.xs{3}, V_final, state(1), state(2), state(3));
% 
% % Display the result
% fprintf('The value function at state [%.2f, %.2f, %.2f] is V = %.4f\n', state(1), state(2), state(3), value_at_state);
% 
% % Save the grid and value function
% save('g.mat', 'g');
% save('V_final.mat', 'V_final');



% % Assuming V and g are loaded in the workspace
% % Extract the last time step of V
% V_final = V(:,:,:,end); 
% 
% % Choose a specific theta index for visualization
% theta_index = round(size(g.vs{3}, 1) / 2); % Middle index in theta dimension
% theta_value = g.vs{3}(theta_index); % Corresponding theta value
% 
% % Extract the slice of V_final at the chosen theta index
% V_slice = V_final(:,:,theta_index);
% 
% % Create a mesh plot
% figure;
% mesh(g.xs{1}(:,:,theta_index), g.xs{2}(:,:,theta_index), V_slice);
% xlabel('x');
% ylabel('y');
% zlabel('V');
% title(['Mesh Plot of V for \theta = ', num2str(theta_value)]);
% colorbar;


% Assuming V and g are loaded in the workspace
% Extract the last time step of V
V_final = V(:,:,:,end); 

% Choose a specific theta index for visualization
theta_index = round(size(g.vs{3}, 1) / 2); % Middle index in theta dimension
theta_value = g.vs{3}(theta_index); % Corresponding theta value

% Extract the slice of V_final at the chosen theta index
V_slice = V_final(:,:,theta_index);

% Create a mesh plot
figure;
mesh(g.xs{1}(:,:,theta_index), g.xs{2}(:,:,theta_index), V_slice);
xlabel('x');
ylabel('y');
zlabel('V');
title(['Mesh Plot of V for \theta = ', num2str(theta_value)]);
colorbar; 
