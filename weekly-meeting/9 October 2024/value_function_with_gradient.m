% Define the range for x1 and x2
x1 = linspace(-5, 5, 20);  % Fewer points for gradient visualization
x2 = linspace(-5, 5, 20);

% Create a meshgrid for x1 and x2
[X1, X2] = meshgrid(x1, x2);

% Define the bowl-shaped function
Z = -(X1.^2 + X2.^2);

% Compute the gradients
[dx, dy] = gradient(Z);

% Plot the 3D surface
figure;
surf(X1, X2, Z);
hold on;

% Add labels and title
xlabel('x1');
ylabel('x2');
zlabel('Value Function');
title('Bowl-Shaped Value Function with Gradient Path');

% Plot the gradient vectors (quiver3 plots the vectors in 3D)
quiver3(X1, X2, Z, -dx, -dy, zeros(size(Z)), 'r', 'LineWidth', 2);

% Add the path that follows the gradient towards the origin
% (Example of a path toward (0, 0))
t = linspace(0, 1, 50);
x_path = -5 * (1 - t);  % Example path from (-5, -5) to (0, 0)
y_path = -5 * (1 - t);
z_path = -(x_path.^2 + y_path.^2);
plot3(x_path, y_path, z_path, 'k', 'LineWidth', 3);

% Enhance the appearance
shading interp;
colormap jet;
colorbar;

hold off;
