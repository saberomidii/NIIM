clear all
clc
close all

% Define system parameters
g = 9.81;    % acceleration due to gravity (m/s^2)
l = 1.0;     % length of the pendulum (m)
m = 5.0;     % mass of the pendulum (kg)
Q = diag([1, 1]);  % state cost matrix
R = 1;            % control cost

% Discretize state space and time
theta_range = linspace(-0.3, 0.3, 100);   % discretize angle
theta_dot_range = linspace(-0.6, 0.6, 100); % discretize angular velocity
u_range = linspace(-1, 1, 100);             % discretize control input
[V_theta, V_theta_dot] = meshgrid(theta_range, theta_dot_range);
time_steps = linspace(0, 1, 60);          % discretize time from 0 to T=10
K_lqr=[40.62 13.69];
% Initialize value function
V = zeros([size(V_theta), length(time_steps)]);

% Set terminal cost (boundary condition)
V(:, :, end) = 0.1;  % Terminal condition V(x, T) = 0 for all states

% Time step size
dt = time_steps(2) - time_steps(1);

% Define value function bounds
V_lower_bound = -10;
V_upper_bound = 10;

% Prepare figure for animation
figure;
h = surf(V_theta, V_theta_dot, V(:, :, end)); 
xlabel('Theta (rad)');
ylabel('Theta dot (rad/s)');
zlabel('Value Function');
title('Optimal Value Function for Inverted Pendulum');

% Dynamic programming to solve HJB with time derivative
for t = length(time_steps)-1:-1:1
    V_old = V(:, :, t+1);  % Previous time step
    
    for i = 1:length(theta_range)
        for j = 1:length(theta_dot_range)
            theta = theta_range(i);
            theta_dot = theta_dot_range(j);
            min_H = V_upper_bound;  % Initialize with the upper bound
            
            % Iterate over control inputs to minimize Hamiltonian
            for k = 1:length(u_range)
                %%% input
                u = u_range(k);
                % u=randn;
                %u=norm([theta,theta_dot]);
                % u=(m*l^2)-g/l * sin(theta)-1;
                % u=K_lqr*[theta;theta_dot];
                % Dynamics of the system
                f = (g/l * sin(theta) + u/(m*l^2) + rand);
                % L = Running Cost Function
                L = [theta; theta_dot]' * Q * [theta; theta_dot] + u^2 * R;
                % L = 0; % Placeholder for the running cost
                
                % Compute Hamiltonian with time derivative term
                H = (1/dt) * (V_old(i, j) - V(i, j, t)) + ...
                    (V_theta(i, j) * theta_dot + V_theta_dot(i, j) * f + L);
                
                if H < min_H
                    min_H = H;
                    optimal_u = u;
                end
            end
            
            % Update value function with bounds
            V(i, j, t) = min(max(min_H * dt + V_old(i, j), V_lower_bound), V_upper_bound);
        end
    end
    
    % Update the plot for animation
    set(h, 'ZData', V(:, :, t));
    title(['Optimal Value Function at Time Step: ', num2str(t)]);
    drawnow;
    
    % Display progress
    if mod(t, 10) == 0
        disp(['Time step: ', num2str(t), ' / ', num2str(length(time_steps))]);
    end
end
