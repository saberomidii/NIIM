clc;clear all
close all% Parameters
mu_true = [5; 3];  % True mean vector [μ₁, μ₂]
Sigma = [4, 1; 1, 2];  % Known covariance matrix Σ
mu0 = [0; 0];  % Prior mean vector μ₀
Sigma0 = [10, 0; 0, 10];  % Prior covariance matrix Σ₀
n = 5000;  % Sample size

% Simulate data
% rng(0);  % For reproducibility
X = mvnrnd(mu_true, Sigma, n)';  % Simulate n samples (each column is a sample)

% Compute sample mean
x_bar = mean(X, 2);  % Sample mean vector x̄

% Compute posterior covariance matrix
Sigma_n_inv = n * inv(Sigma) + inv(Sigma0);  % Compute inverse of posterior covariance
Sigma_n = inv(Sigma_n_inv);  % Posterior covariance matrix Σₙ

% Compute posterior mean vector
mu_n = Sigma_n * (n * inv(Sigma) * x_bar + inv(Sigma0) * mu0);  % Posterior mean vector μₙ

% Display results
disp('Prior Mean (μ₀):');
disp(mu0);
disp('Prior Covariance Matrix (Σ₀):');
disp(Sigma0);
disp('Sample Mean (x̄):');
disp(x_bar);
disp('Posterior Mean (μₙ):');
disp(mu_n);
disp('Posterior Covariance Matrix (Σₙ):');
disp(Sigma_n);

% Plotting
% Create grid for plotting
x1 = linspace(mu_true(1) - 5, mu_true(1) + 5, 100);  % Range for μ₁
x2 = linspace(mu_true(2) - 5, mu_true(2) + 5, 100);  % Range for μ₂
[X1, X2] = meshgrid(x1, x2);  % Create meshgrid for μ₁ and μ₂
grid_points = [X1(:), X2(:)]';  % Grid points for evaluation

% Compute densities
prior_pdf = mvnpdf(grid_points', mu0', Sigma0);  % Prior PDF evaluated at grid points
likelihood_pdf = mvnpdf(grid_points', x_bar', Sigma / n);  % Likelihood PDF
posterior_pdf = mvnpdf(grid_points', mu_n', Sigma_n);  % Posterior PDF

% Reshape for surface plot
prior_pdf = reshape(prior_pdf, size(X1));  % Reshape prior PDF for plotting
likelihood_pdf = reshape(likelihood_pdf, size(X1));  % Reshape likelihood PDF
posterior_pdf = reshape(posterior_pdf, size(X1));  % Reshape posterior PDF

% Plot 3D surfaces
figure('Position', [100, 100, 1800, 500]);

% Prior Distribution Plot
subplot(1,3,1);
surf(X1, X2, prior_pdf, 'EdgeColor', 'none');
title('Prior Distribution');
xlabel('\mu_1');
ylabel('\mu_2');
zlabel('Density');
axis tight;
grid on;
view(-30, 30);  % Adjust view angle
colorbar;
hold on;
scatter3(mu0(1), mu0(2), max(prior_pdf(:)), 100, 'k', 'filled', 'MarkerEdgeColor', 'k');
text(mu0(1), mu0(2), max(prior_pdf(:)), ' \leftarrow Prior Mean', 'FontSize', 12);

% Likelihood Function Plot
subplot(1,3,2);
surf(X1, X2, likelihood_pdf, 'EdgeColor', 'none');
title('Likelihood Function');
xlabel('\mu_1');
ylabel('\mu_2');
zlabel('Density');
axis tight;
grid on;
view(-30, 30);
colorbar;
hold on;
scatter3(x_bar(1), x_bar(2), max(likelihood_pdf(:)), 100, 'k', 'filled', 'MarkerEdgeColor', 'k');
text(x_bar(1), x_bar(2), max(likelihood_pdf(:)), ' \leftarrow Sample Mean', 'FontSize', 12);

% Posterior Distribution Plot
subplot(1,3,3);
surf(X1, X2, posterior_pdf, 'EdgeColor', 'none');
title('Posterior Distribution');
xlabel('\mu_1');
ylabel('\mu_2');
zlabel('Density');
axis tight;
grid on;
view(-30, 30);
colorbar;
hold on;
scatter3(mu_n(1), mu_n(2), max(posterior_pdf(:)), 100, 'k', 'filled', 'MarkerEdgeColor', 'k');
text(mu_n(1), mu_n(2), max(posterior_pdf(:)), ' \leftarrow Posterior Mean', 'FontSize', 12);

% Adjust color mapping for better visualization
colormap jet;
