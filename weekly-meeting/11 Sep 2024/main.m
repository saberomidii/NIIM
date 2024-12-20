clear 
clc
close all


% MATLAB Code to Visualize -||y - x|| for y Outside the Feasible Set

% Define the set A
A = [-1, 1];

% Define the point x = 0.2
x = 0.2;

% Generate points outside the feasible set (extending range beyond [-1, 1])
y_out = linspace(-2, 2, 100); % Points between -2 and 2

% Calculate the signed distance ||y - x|| (positive inside the set, negative outside)
norm_yx_out = abs(y_out - x);
signed_distance = norm_yx_out;  % Initialize with the same values
signed_distance(y_out < A(1) | y_out > A(2)) = -norm_yx_out(y_out < A(1) | y_out > A(2)); % Negative norm outside the feasible set

% Find the -infimum of signed distance, which occurs for the largest negative value
[inf_val, inf_idx] = min(signed_distance); % Negative infimum value and its index
y_inf = y_out(inf_idx); % The y-value where negative infimum occurs

% Create a figure for visualization
figure;
hold on;

% Plot the signed distance ||y - x|| as a function of y (negative outside feasible set)
plot(y_out, signed_distance, 'b-', 'LineWidth', 2);

% Plot a dashed red line for the interval [-1, 1] to represent the feasible set boundaries
plot([A(1), A(1)], [-2, 2], 'r--', 'LineWidth', 2); % Left boundary at y = -1
plot([A(2), A(2)], [-2, 2], 'r--', 'LineWidth', 2); % Right boundary at y = 1

% Plot a blue point at x = 0.2 (time zero)
plot(x, 0, 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b'); % Blue point at x = 0.2

% Mark the -infimum on the plot with a green star
plot(y_inf, inf_val, 'g*', 'MarkerSize', 10, 'MarkerFaceColor', 'g'); % Green star for -infimum

% Labels and title
xlabel('y');
ylabel('Signed ||y - x||');
title('Signed Norm ||y - x|| for x = 0.2 including -infimum outside the Feasible Set A = [-1, 1]');
xlim([-2 2]);
ylim([-2 2]);
grid on;
hold off;
