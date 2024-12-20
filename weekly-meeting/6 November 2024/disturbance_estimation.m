% Clear previous figures and variables
clear;
close all;
clc;

% Define the size of the rectangle (Feasible Set)
width = 2;
height = 1;

% Calculate the lower-left corner position
x_left = -width/2;
y_bottom = -height/2;

% Define standard deviations for the prior distribution
sigma_x = 0.5;
sigma_y = 0.25;

% Prior mean and covariance
mu0 = [0; 0];
Sigma0 = [sigma_x^2, 0; 0, sigma_y^2];

% Likelihood (Observation) parameters
mu1 = [-0.75; 3];
sigma1 = 4;
Sigma1 = [sigma1^2, 0; 0, sigma1^2];

% Compute the posterior covariance
Sigma_post_inv = inv(Sigma0) + inv(Sigma1);
Sigma_post = inv(Sigma_post_inv);

% Compute the posterior mean
mu_post = Sigma_post * (inv(Sigma0) * mu0 + inv(Sigma1) * mu1);

% Create a grid that covers both the feasible set and the new mean
x_range = linspace(min(x_left, mu1(1) - 3*sigma1), max(x_left + width, mu1(1) + 3*sigma1), 200);
y_range = linspace(min(y_bottom, mu1(2) - 3*sigma1), max(y_bottom + height, mu1(2) + 3*sigma1), 200);
[X, Y] = meshgrid(x_range, y_range);

% Compute the posterior probability density function
Z_post = zeros(size(X));
for i = 1:size(X,1)
    for j = 1:size(X,2)
        x_vec = [X(i,j); Y(i,j)];
        Z_post(i,j) = mvnpdf(x_vec', mu_post', Sigma_post);
    end
end

% Plot the feasible set (rectangle)
figure;
rectangle('Position', [x_left, y_bottom, width, height], 'EdgeColor', 'b', 'LineWidth', 2);
hold on;

% Plot the posterior distribution
surf(X, Y, Z_post, 'EdgeColor', 'none', 'FaceAlpha', 0.7);

% Mark the prior mean, observation mean, and posterior mean
plot3(mu0(1), mu0(2), max(Z_post(:)), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
text(mu0(1), mu0(2), max(Z_post(:)), '  Prior Mean', 'VerticalAlignment', 'bottom');

plot3(mu1(1), mu1(2), max(Z_post(:)), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
text(mu1(1), mu1(2), max(Z_post(:)), '  Observation Mean', 'VerticalAlignment', 'bottom');

plot3(mu_post(1), mu_post(2), max(Z_post(:)), 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k');
text(mu_post(1), mu_post(2), max(Z_post(:)), '  Posterior Mean', 'VerticalAlignment', 'bottom');

% Customize the colormap and add a colorbar
colormap jet;
colorbar;

% Label the axes
xlabel('x');
ylabel('y');
zlabel('Probability Density');

% Add a title to the plot
title('Posterior Distribution after Bayesian Update');

% Set the viewing angle for better visualization
view(45, 30);

% Turn on the grid
grid on;

% Adjust axis limits for better visualization
xlim([min(x_left, mu1(1) - 3*sigma1), max(x_left + width, mu1(1) + 3*sigma1)]);
ylim([min(y_bottom, mu1(2) - 3*sigma1), max(y_bottom + height, mu1(2) + 3*sigma1)]);

% Release the hold
hold off;
