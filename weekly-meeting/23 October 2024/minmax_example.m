% Objective function for max-min optimization
f = @(x, y) (x - y).^2;

% Constraints for x and y
lb_x = 0;  % Lower bound for x
ub_x = 1;  % Upper bound for x
lb_y = 0;  % Lower bound for y
ub_y = 1;  % Upper bound for y

% Initial guesses
x0 = 0.5;  % Initial guess for x
y0 = 0.5;  % Initial guess for y

% Maximize over y for a fixed x
maximize_y = @(x) fminbnd(@(y) -f(x, y), lb_y, ub_y);  % Negate to maximize

% Minimize over x by first maximizing over y
minimize_x = @(x) -maximize_y(x);  % We negate because fminbnd minimizes

% Define options for fminbnd (minimization over x)
options = optimset('TolX', 1e-6);

% Solve for x using fminbnd
x_opt = fminbnd(minimize_x, lb_x, ub_x, options);

% Get the optimal y corresponding to x_opt
y_opt = fminbnd(@(y) -f(x_opt, y), lb_y, ub_y);

% Display the results
fprintf('Optimal x: %f\n', x_opt);
fprintf('Optimal y: %f\n', y_opt);
fprintf('Optimal value of the function: %f\n', f(x_opt, y_opt));

% Create a mesh grid for plotting
[x_grid, y_grid] = meshgrid(linspace(lb_x, ub_x, 100), linspace(lb_y, ub_y, 100));

% Calculate the function values over the grid
f_vals = f(x_grid, y_grid);

% Plot the function
figure;
mesh(x_grid, y_grid, f_vals);
hold on;

% Plot the optimal point
plot3(x_opt, y_opt, f(x_opt, y_opt), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
title('Max-Min Optimization Problem');
xlabel('x');
ylabel('y');
zlabel('f(x, y)');
colorbar;
grid on;

% Add contours for better visualization
figure;
contour(x_grid, y_grid, f_vals, 50);
hold on;
plot(x_opt, y_opt, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
title('Contour Plot of f(x, y)');
xlabel('x');
ylabel('y');
colorbar;
grid on;
