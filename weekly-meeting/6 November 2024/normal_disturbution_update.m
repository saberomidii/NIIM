% Parameters
mu_true = 5;        % True mean
sigma2 = 4;         % Known variance
mu0 = 0;            % Prior mean
tau0_sq = 10;       % Prior variance
n = 20;             % Sample size

% Simulate data
rng(0);  % For reproducibility
x = mu_true + sqrt(sigma2) * randn(n,1);

% Compute sample mean
x_bar = mean(x);

% Compute posterior variance
tau_n_sq = 1 / (n / sigma2 + 1 / tau0_sq);

% Compute posterior mean
mu_n = tau_n_sq * (n * x_bar / sigma2 + mu0 / tau0_sq);

% Display results
fprintf('Prior Mean: %.2f, Prior Variance: %.2f\n', mu0, tau0_sq);
fprintf('Sample Mean: %.2f\n', x_bar);
fprintf('Posterior Mean: %.2f, Posterior Variance: %.2f\n', mu_n, tau_n_sq);

% Plotting
mu_values = linspace(mu_true - 3*sqrt(sigma2), mu_true + 3*sqrt(sigma2), 1000);

% Prior distribution
prior_pdf = normpdf(mu_values, mu0, sqrt(tau0_sq));

% Likelihood (scaled)
likelihood_pdf = normpdf(mu_values, x_bar, sqrt(sigma2 / n));
likelihood_pdf = likelihood_pdf / max(likelihood_pdf) * max(prior_pdf);

% Posterior distribution
posterior_pdf = normpdf(mu_values, mu_n, sqrt(tau_n_sq));

% Plot
figure;
plot(mu_values, prior_pdf, 'b-', 'LineWidth', 2); hold on;
plot(mu_values, likelihood_pdf, 'g--', 'LineWidth', 2);
plot(mu_values, posterior_pdf, 'r-', 'LineWidth', 2);
legend('Prior', 'Likelihood', 'Posterior');
xlabel('\mu');
ylabel('Density');
title('Bayesian Update of Normal Mean with Known Variance');
grid on;
