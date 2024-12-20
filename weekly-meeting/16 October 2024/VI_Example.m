% Variational Inequality Example: Obstacle Problem

% Parameters
a = 0;          % Left boundary
b = 1;          % Right boundary
N = 100;        % Number of interior points
h = (b - a) / (N + 1);  % Mesh size
x = linspace(a + h, b - h, N)';  % Discrete spatial points

% Obstacle Function phi(x)
phi = 0.2 * sin(2 * pi * x);  % Example obstacle

% Source Function f(x)
f = ones(N, 1);  % Constant source term

% Construct the Tridiagonal Matrix A
e = ones(N, 1);
A = (1 / h^2) * spdiags([e -2*e e], -1:1, N, N);

% Boundary Conditions (u(0) = u(1) = 0)
% Implicitly enforced by modifying A and f if needed

% Initial Guess
u = max(phi, 0);  % Ensure initial guess satisfies u >= phi

% Tolerance and Maximum Iterations
tol = 1e-6;
maxIter = 1000;

% Projected Successive Over-Relaxation (PSOR) Method
omega = 1.5;  % Relaxation parameter
for iter = 1:maxIter
    u_old = u;
    for i = 1:N
        % Compute the residual
        res = f(i) - A(i, :) * u;
        % Update u(i)
        u_new = u(i) + omega * res / A(i, i);
        % Apply the obstacle constraint
        u(i) = max(u_new, phi(i));
    end
    % Check for convergence
    if norm(u - u_old, inf) < tol
        fprintf('Converged in %d iterations.\n', iter);
        break;
    end
end

% Plotting the Solution and the Obstacle
figure;
plot(x, u, 'b-', 'LineWidth', 2); hold on;
plot(x, phi, 'r--', 'LineWidth', 2);
xlabel('x');
ylabel('u(x)');
title('Solution of the Obstacle Problem');
legend('Computed Solution u(x)', 'Obstacle \phi(x)');
grid on;
