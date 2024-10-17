% MATLAB Code to Set Up Gaussian Process Regression for a Unit Circle with 3D Visualization

% Clear workspace and command window
clear; clc; close all;

%% 1. Define the Mean and Covariance Functions as in the Paper

% Mean function (assumed zero, but defined explicitly)
mu = @(x) zeros(size(x,1),1);

% Covariance kernel function (Squared Exponential Kernel with hyperparameters)
% Hyperparameters: signal variance (sigma_f) and length-scale (l)
sigma_f = 1.0;  % Signal variance (to be optimized/refitted)
l = 0.5;        % Length-scale (to be optimized/refitted)
k = @(x1, x2) sigma_f^2 * exp(-0.5 * pdist2(x1, x2, 'euclidean').^2 / l^2);

%% 2. Generate Data Representing the Unit Circle

% Number of training data points
N = 100;

% Generate random points in the 2D plane
rng(1);  % For reproducibility
X = 2 * rand(N, 2) - 1;  % Uniformly distributed in [-1, 1] x [-1, 1]

% True function representing the unit circle
z_true = sum(X.^2, 2) - 1;  % z(x, y) = x^2 + y^2 - 1

% Add Gaussian noise to the observations
noise_std = 0.05;  % Standard deviation of noise
epsilon = noise_std * randn(size(z_true));  % Gaussian noise
z_noisy = z_true + epsilon;  % Noisy observations

%% 3. Compute the Posterior Mean and Covariance at Query Points

% Query points where we want to predict the function
grid_size = 50;
[x1_grid, x2_grid] = meshgrid(linspace(-1.5, 1.5, grid_size), linspace(-1.5, 1.5, grid_size));
X_star = [x1_grid(:), x2_grid(:)];  % Flatten the grid for computation

% Compute covariance matrices

% K(X, X)
K_XX = k(X, X);

% K(X_star, X)
K_XstarX = k(X_star, X);

% K(X_star, X_star)
K_XstarXstar_diag = diag(k(X_star, X_star));  % Only need the diagonal

% Add noise variance to the diagonal of K_XX
sigma_n_sq = noise_std^2;
K_XX_noisy = K_XX + sigma_n_sq * eye(N);

% Compute the inverse of the noisy covariance matrix
% For numerical stability, use Cholesky decomposition
L = chol(K_XX_noisy + 1e-6 * eye(N), 'lower');  % Add small value to diagonal for stability
alpha = L'\(L\ (z_noisy - mu(X)));

% Equation (10a): Posterior mean
E_z_Xstar = mu(X_star) + K_XstarX * alpha;

% Equation (10b): Posterior variance
v = L\K_XstarX';
var_z_Xstar = K_XstarXstar_diag - sum(v.^2, 1)';
std_z_Xstar = sqrt(var_z_Xstar);

%% 4. Visualize the Results in 3D

% Reshape the outputs for plotting
E_z_Xstar_grid = reshape(E_z_Xstar, grid_size, grid_size);
std_z_Xstar_grid = reshape(std_z_Xstar, grid_size, grid_size);

% Plot the posterior mean surface
figure;
surf(x1_grid, x2_grid, E_z_Xstar_grid, 'EdgeColor', 'none');
colormap jet;
colorbar;
xlabel('x');
ylabel('y');
zlabel('Posterior Mean z(x, y)');
title('Gaussian Process Posterior Mean Surface');
view(3);
grid on;

% Plot the posterior variance surface
figure;
surf(x1_grid, x2_grid, std_z_Xstar_grid, 'EdgeColor', 'none');
colormap jet;
colorbar;
xlabel('x');
ylabel('y');
zlabel('Posterior Standard Deviation');
title('Gaussian Process Posterior Standard Deviation Surface');
view(3);
grid on;

% Overlay the unit circle on the mean surface
figure;
surf(x1_grid, x2_grid, E_z_Xstar_grid, 'EdgeColor', 'none');
colormap jet;
colorbar;
hold on;
% Plot the contour where z = 0 (estimated unit circle)
contour3(x1_grid, x2_grid, E_z_Xstar_grid, [0, 0], 'k-', 'LineWidth', 2);
% Plot the true unit circle
theta = linspace(0, 2*pi, 200);
plot3(cos(theta), sin(theta), zeros(size(theta)), 'r--', 'LineWidth', 2);
xlabel('x');
ylabel('y');
zlabel('Posterior Mean z(x, y)');
title('Posterior Mean Surface with Estimated and True Unit Circle');
view(3);
grid on;
hold off;

% Plot the posterior mean with uncertainty bounds (mean ± 2σ)
figure;
surf(x1_grid, x2_grid, E_z_Xstar_grid, 'EdgeColor', 'none');
colormap jet;
colorbar;
hold on;
% Upper bound surface (mean + 2σ)
surf(x1_grid, x2_grid, E_z_Xstar_grid + 2 * std_z_Xstar_grid, 'FaceAlpha', 0.5, 'EdgeColor', 'none');
% Lower bound surface (mean - 2σ)
surf(x1_grid, x2_grid, E_z_Xstar_grid - 2 * std_z_Xstar_grid, 'FaceAlpha', 0.5, 'EdgeColor', 'none');
xlabel('x');
ylabel('y');
zlabel('Posterior Mean ± 2σ');
title('Posterior Mean Surface with Confidence Bounds');
view(3);
grid on;
hold off;

%% 5. Visualize the Normal Distribution Changing Across the Domain

% For selected points, plot the normal distribution of z(x, y)
selected_points = [0, 0; 0.5, 0.5; 1.0, 0.0; -0.75, -0.75];
figure;
for i = 1:size(selected_points, 1)
    x_sel = selected_points(i, :);
    % Find the index of the closest point in the grid
    [~, idx] = min(sum((X_star - x_sel).^2, 2));
    mu_sel = E_z_Xstar(idx);
    sigma_sel = std_z_Xstar(idx);
    % Plot the normal distribution at this point
    subplot(2, 2, i);
    x_vals = linspace(mu_sel - 3*sigma_sel, mu_sel + 3*sigma_sel, 100);
    y_vals = normpdf(x_vals, mu_sel, sigma_sel);
    plot(x_vals, y_vals, 'b-', 'LineWidth', 2);
    title(sprintf('Normal Distribution at (%.2f, %.2f)', x_sel(1), x_sel(2)));
    xlabel('z(x, y)');
    ylabel('Probability Density');
    grid on;
end

% Adjust subplot spacing
tight_layout();

%% Function to Adjust Subplot Spacing (Optional)
function tight_layout()
    % Adjusts subplot spacing to minimize whitespace
    % Get all axes handles
    ax = findall(gcf, 'type', 'axes');
    % Remove labels to avoid overlap
    for i = 1:length(ax)
        ax(i).Title.Units = 'normalized';
        ax(i).Title.Position(2) = 1.05;
    end
    % Adjust positions
    for i = 1:length(ax)
        ax_pos = get(ax(i), 'Position');
        ax_pos(3) = 0.4;  % width
        ax_pos(4) = 0.4;  % height
        set(ax(i), 'Position', ax_pos);
    end
end
