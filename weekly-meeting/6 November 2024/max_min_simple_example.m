clear all
clc


% Define the function f(A, B)
f = @(A, B) (A - 2).^2 + (B + 3).^2;

% Range of A values to test
A_values = -5:0.1:5;

% Initialize variables to store results
min_max_value = Inf; % Start with a large value for minimum of max
optimal_A = NaN;

% Loop over each A to find the corresponding B that maximizes f(A, B)
for A = A_values
    % Define the inner maximization problem for B
    B_values = -5:0.01:5; % Range of B values
    f_values = f(A, B_values); % Calculate f(A, B) for fixed A and varying B
    [max_value, max_index] = max(f_values); % Find max over B
    
    % Check if this max value is the minimum encountered so far
    if max_value < min_max_value
        min_max_value = max_value;
        optimal_A = A;
        optimal_B = B_values(max_index);
    end
end

% Display the optimal results
fprintf('Optimal A that minimizes max value of f(A, B): %.2f\n', optimal_A);
fprintf('Corresponding B that maximizes f(A, B): %.2f\n', optimal_B);
fprintf('Minimum of the maximum values: %.2f\n', min_max_value);

% Plot the function f(A, B) for visualization
[A_grid, B_grid] = meshgrid(-5:0.1:5, -5:0.1:5);
f_grid = f(A_grid, B_grid);
figure;
surf(A_grid, B_grid, f_grid);
xlabel('A');
ylabel('B');
zlabel('f(A, B)');
title('Min-Max Optimization Visualization');
hold on;
plot3(optimal_A, optimal_B, min_max_value, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
hold off;
