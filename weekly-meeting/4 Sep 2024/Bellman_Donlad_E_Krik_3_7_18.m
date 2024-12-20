clear all
clc

% Parameters
delta_t = 1;  % Time step size

% Terminal cost at N (given as 1.12)
terminal_cost = 1.12;

% Step N (Final Time Step)
x_current = 4.5;  % Example state at time step N
u_current = 0.5;  % Example control at time step N

% Calculate the total cost at N
quadratic_cost_N = x_current^2 + u_current^2;
total_cost_N = quadratic_cost_N + terminal_cost;

% Display results for N
fprintf('Step N:\n');
fprintf('Current state x(N) = %.2f\n', x_current);
fprintf('Control u(N) = %.2f\n', u_current);
fprintf('Quadratic cost g_D(x(N), u(N)) = %.2f\n', quadratic_cost_N);
fprintf('Total cost at N (including terminal cost) = %.2f\n\n', total_cost_N);

% Step N-1
x_current = 2;  % Example state at time step N-1
u_current = 0.5;  % Example control at time step N-1

% Calculate the next state using the state transition equation (Eq. 3.7-4)
x_next_N1 = x_current + delta_t * (x_current + u_current);

% Calculate the quadratic cost at N-1
quadratic_cost_N1 = x_current^2 + u_current^2;

% Calculate the total cost at N-1
total_cost_N1 = quadratic_cost_N1 + total_cost_N;

% Display results for N-1
fprintf('Step N-1:\n');
fprintf('Current state x(N-1) = %.2f\n', x_current);
fprintf('Control u(N-1) = %.2f\n', u_current);
fprintf('Next state x(N) = %.2f\n', x_next_N1);
fprintf('Quadratic cost g_D(x(N-1), u(N-1)) = %.2f\n', quadratic_cost_N1);
fprintf('Total cost at N-1 (including cost from N) = %.2f\n\n', total_cost_N1);

% Step N-2
x_current = x_next_N1;  % Using the next state from N-1 as the current state for N-2
u_current = 0.5;  % Example control at time step N-2

% Calculate the next state using the state transition equation (Eq. 3.7-4)
x_next_N2 = x_current + delta_t * (x_current + u_current);

% Calculate the quadratic cost at N-2
quadratic_cost_N2 = x_current^2 + u_current^2;

% Calculate the total cost at N-2
total_cost_N2 = quadratic_cost_N2 + total_cost_N1;

% Display results for N-2
fprintf('Step N-2:\n');
fprintf('Current state x(N-2) = %.2f\n', x_current);
fprintf('Control u(N-2) = %.2f\n', u_current);
fprintf('Next state x(N-1) = %.2f\n', x_next_N2);
fprintf('Quadratic cost g_D(x(N-2), u(N-2)) = %.2f\n', quadratic_cost_N2);
fprintf('Total cost at N-2 (including cost from N-1) = %.2f\n\n', total_cost_N2);

% Step N-3
x_current = x_next_N2;  % Using the next state from N-2 as the current state for N-3
u_current = 0.5;  % Example control at time step N-3

% Calculate the next state using the state transition equation (Eq. 3.7-4)
x_next_N3 = x_current + delta_t * (x_current + u_current);

% Calculate the quadratic cost at N-3
quadratic_cost_N3 = x_current^2 + u_current^2;

% Calculate the total cost at N-3
total_cost_N3 = quadratic_cost_N3 + total_cost_N2;

% Display results for N-3
fprintf('Step N-3:\n');
fprintf('Current state x(N-3) = %.2f\n', x_current);
fprintf('Control u(N-3) = %.2f\n', u_current);
fprintf('Next state x(N-2) = %.2f\n', x_next_N3);
fprintf('Quadratic cost g_D(x(N-3), u(N-3)) = %.2f\n', quadratic_cost_N3);
fprintf('Total cost at N-3 (including cost from N-2) = %.2f\n\n', total_cost_N3);
