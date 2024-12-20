clear all
clc
close all

% Generate training data
n = 10;
X = linspace(0, 5, n)';            % Training inputs
f = sin(X);                        % True function
sigma_n = 0.1;                     % Noise standard deviation
y = f + sigma_n * randn(size(X));  % Noisy observations

% Generate test data
X_star = linspace(0, 5, 100)';     % Test inputs


% Hyperparameters
sigma_f = 1;   % Signal variance
ell = 1;       % Length scale

% Covariance function
covSE = @(x, x_prime) sigma_f^2 * exp(-0.5 * (x - x_prime').^2 / ell^2);


% Compute covariance matrices
K = covSE(X, X) + sigma_n^2 * eye(n);        % Training covariance matrix
K_s = covSE(X, X_star);                      % Cross-covariance matrix
K_ss = covSE(X_star, X_star);                % Test covariance matrix


% Compute the inverse of K
K_inv = inv(K);

% Predictive mean
mu_s = K_s' * K_inv * y;

% Predictive covariance
cov_s = K_ss - K_s' * K_inv * K_s;

% Standard deviation
sigma_s = sqrt(diag(cov_s));


% Compute the inverse of K
K_inv = inv(K);

% Predictive mean
mu_s = K_s' * K_inv * y;

% Predictive covariance
cov_s = K_ss - K_s' * K_inv * K_s;

% Standard deviation
sigma_s = sqrt(diag(cov_s));

% Plot training data
figure;
hold on;
plot(X, y, 'kx', 'MarkerSize', 10, 'LineWidth', 2,"Color","Yellow");

% Plot predictive mean
plot(X_star, mu_s, 'b-', 'LineWidth', 2);

% Plot confidence intervals
fill([X_star; flipud(X_star)], [mu_s + 2*sigma_s; flipud(mu_s - 2*sigma_s)], [7 7 7]/8, 'EdgeColor', 'green','FaceAlpha', 0.3);

% True function
plot(X_star, sin(X_star), 'r--', 'LineWidth', 2);

xlabel('Input, x');
ylabel('Output, f(x)');
title('Gaussian Process Regression');
legend('Training Data', 'Predictive Mean', 'Confidence Interval', 'True Function');
grid on;
hold off;
