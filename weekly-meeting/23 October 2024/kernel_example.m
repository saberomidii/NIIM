function [K] = squared_exponential_kernel(X, Xp, sigma_f, L)
    % Input:
    % X    : NxD matrix of N input points (each row is an input point with D dimensions)
    % Xp   : MxD matrix of M input points (each row is an input point with D dimensions)
    % sigma_f : Signal variance
    % L    : DxD diagonal matrix of length scales for each dimension

    % Output:
    % K    : NxM covariance matrix
    
    N = size(X, 1); % Number of points in X
    M = size(Xp, 1); % Number of points in Xp
    D = size(X, 2); % Dimension of the input space
    
    % Preallocate covariance matrix
    K = zeros(N, M);
    
    % Loop over all pairs of input points in X and Xp
    for i = 1:N
        for j = 1:M
            % Compute the difference between the two points
            diff = X(i, :) - Xp(j, :);
            
            % Compute the kernel using the squared exponential formula
            K(i, j) = sigma_f^2 * exp(-0.5 * diff * (L \ diff'));
        end
    end
end

% Define input points X and test points Xp
X = [1; 2; 3];  % Input points (1D case)
Xp = linspace(0, 4, 100)';  % Test points for posterior estimation

% Define the hyperparameters
sigma_f = 1.0;  % Signal variance
L = diag([1.0]);  % Length scale (1D input, hence a single length scale)
sigma_n = 0.1;  % Noise variance

% Compute the covariance matrices
K = squared_exponential_kernel(X, X, sigma_f, L);  % Covariance between training points
K_s = squared_exponential_kernel(X, Xp, sigma_f, L);  % Cross-covariance between training and test points
K_ss = squared_exponential_kernel(Xp, Xp, sigma_f, L);  % Covariance of test points
K_noise = K + sigma_n^2 * eye(size(K));  % Add noise to the covariance of the training points

% Define some example observed data with noise
Y = sin(X) + 0.1 * randn(size(X));  % Observed outputs with noise

% Compute the posterior mean and covariance
K_inv = inv(K_noise);  % Inverse of the noisy training covariance matrix
mu_s = K_s' * K_inv * Y;  % Posterior mean
cov_s = K_ss - K_s' * K_inv * K_s;  % Posterior covariance

% Plot the GP prior (mean = 0, variance)
figure;
subplot(2, 1, 1);
plot(Xp, zeros(size(Xp)), 'k--', 'LineWidth', 1.5);  % Prior mean (zero)
hold on;
fill([Xp; flipud(Xp)], [2*sqrt(diag(K_ss)); -2*flipud(sqrt(diag(K_ss)))], [7 7 7]/8, 'FaceAlpha', 0.5);  % Prior variance (95% confidence interval)
title('GP Prior');
xlabel('x');
ylabel('f(x)');
legend('Mean', 'Confidence Interval');

% Plot the GP posterior (mean and variance)
subplot(2, 1, 2);
plot(Xp, mu_s, 'b-', 'LineWidth', 1.5);  % Posterior mean
hold on;
fill([Xp; flipud(Xp)], [mu_s + 2*sqrt(diag(cov_s)); flipud(mu_s - 2*sqrt(diag(cov_s)))], [7 7 7]/8, 'FaceAlpha', 0.5);  % Posterior variance (95% confidence interval)
plot(X, Y, 'rx', 'MarkerSize', 8, 'LineWidth', 1.5);  % Observed data points
title('GP Posterior');
xlabel('x');
ylabel('f(x)');
legend('Mean', 'Confidence Interval', 'Observed Data');
