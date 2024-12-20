clear all
clc
close all

% Example parameters
a = 0.137;  % Example scalar value for a
b = 0.274;  % Example scalar value for b
clear all
clc
close all

% Define the range for x
x1_min = -0.3;
x1_max = 0.3;
x2_min = -0.6;
x2_max = 0.6;

% Example parameters
a = 0.137;  % Example scalar value for a
b = 0.274;  % Example scalar value for b

% Create a grid of x values within the specified range
[x1, x2] = meshgrid(linspace(x1_min, x1_max, 100), linspace(x2_min, x2_max, 100));

% Initialize matrix to store h values
h_values = zeros(size(x1));

% Calculate h for each point in the grid
for i = 1:numel(x1)
    x = [x1(i); x2(i)];
    h_values(i) = hS_function(x, a, b);
end

% Plot the results
figure;
surf(x1, x2, h_values);
xlabel('x1');
ylabel('x2');
zlabel('h_S(x)');
title('h_S(x) over the range of x');
hold on;

% Visualize the region where h_S(x) >= 0
contour3(x1, x2, h_values, [0 0], 'LineWidth', 3, 'LineColor', 'r');
hold off;

% Create a new figure for the set where h_S(x) >= 0
figure;
hold on;
xlabel('x1');
ylabel('x2');
title('$$Set S: h_S(x) super zero level set $$','Interpreter', 'latex');

% Plot the points where h_S(x) >= 0
for i = 1:numel(x1)
    if h_values(i) >= 0
        plot(x1(i), x2(i), 'b.');
    end
end

% Add grid and axis labels
grid on;
hold off;

% System parameters
m = 2.0; % mass (kg)
l = 1.0; % length (m)
g = 10; % gravitational constant (m/s^2)
K = [40.62, 13.69]; % Feedback gain matrix

% Desired control input function
u_des = @(t, x) (t < 2) * 3 + (t >= 2 & t < 4) * (-3) + (t >= 4 & t < 6) * 3 + ...
    (t >= 6) * (m * l^2 * (-g/l * sin(x(1)) - [1.5, 1.5] * x));

% Initial state
x0 = [0.1; 0]; % Initial state [theta, theta_dot]

% Simulation time
tspan = [0 10]; % Time span for simulation



% Controller function
controller = @(t, x) (hS_function(x, a, b) <= 0 || norm(u_des(t, x)) > 3) * (-K * x) + ...
                    (hS_function(x, a, b) > 0) * u_des(t, x);

% System dynamics function
dynamics = @(t, x) [x(2); (g/l) * sin(x(1)) + controller(t, x) / (m * l^2)];

% Solve the system using ODE solver
[t, x] = ode45(dynamics, tspan, x0);

% Control input over time
u = arrayfun(@(i) controller(t(i), x(i,:)'), 1:length(t), 'UniformOutput', false);
u = cell2mat(u');

% Plot the state and control input versus time
figure;
subplot(3, 1, 1);
plot(t, x(:, 1), 'b', 'LineWidth', 2);
xlabel('');
ylabel('$$\theta \, (rad)$$', 'Interpreter', 'latex');
grid on;

subplot(3, 1, 2);
plot(t, x(:, 2), 'r', 'LineWidth', 2);
xlabel('');
ylabel('$$\dot{\theta} (rad/s)$$', 'Interpreter', 'latex');
grid on;

subplot(3, 1, 3);
plot(t, u, 'k', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Control Input u (N.m)');
grid on;


% Define inequality constraints: A * x <= b
% These values are approximations based on the red boundary in the image
A = [-1.14, -0.3; 1, 0.3; 0, 1; 0, -1];
b = [0.16; 0.12; 0.6; 0.6];

% Calculate constraints for each point
constraints = zeros(length(x(:,1)'), length(b));
for i = 1:length(b)
    constraints(:,i) = A(i,1) * x(:,1)' + A(i,2) * x(:,2)' - b(i);
end

% Find the points that satisfy all inequalities
% satisfies_constraints = all(constraints <= 0, 2);

% Plot the points and highlight the constrained region
figure;
hold on;


% Plot the zero level set of the h function
contour(x1, x2, h_values, [0 0], 'LineWidth', 3, 'LineColor', 'r');

% Plot the state trajectory
plot(x(:, 1), x(:, 2), 'b', 'LineWidth', 2);

% Set axis limits
xlim([-0.3, 0.3]);
ylim([-0.6, 0.6]);

% Label and style the plot
xlabel('$$\theta \, (rad)$$', 'Interpreter', 'latex');
ylabel('$$\dot{\theta} \, (rad/s)$$', 'Interpreter', 'latex');
title('State Trajectory with Constraints', 'Interpreter', 'latex');
grid on;
hold off;

function hS = hS_function(x, a, b)
% x should be a column vector
% a and b are scalars

% Construct the matrix
A = [1/a^2, 0.5/(a*b); 0.5/(a*b), 1/b^2];

% Calculate the value of the function
hS = 1 - x' * A * x;
end
