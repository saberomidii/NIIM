% MATLAB code to visualize the dynamical system dx/dt = -x^3

% Clear workspace and figures
clear all;
close all;
clc;

%% Define the system
% Vector field function
f = @(x) -x.^3;

% Define the range of x for plotting
x_min = -2;
x_max = 2;
x = linspace(x_min, x_max, 400);

% Compute the vector field
dxdt = f(x);

%% Plot the vector field
figure;
subplot(2,1,1);
plot(x, dxdt, 'b', 'LineWidth', 2);
hold on;
plot([-1, -1], [min(dxdt), max(dxdt)], 'k--'); % Boundary at x = -1
plot([1, 1], [min(dxdt), max(dxdt)], 'k--');   % Boundary at x = 1
xlabel('x');
ylabel('dx/dt');
title('Vector Field f(x) = -x^3');
grid on;

% Indicate the set K = [-1, 1]
fill([-1, 1, 1, -1], [min(dxdt), min(dxdt), max(dxdt), max(dxdt)], [0.9, 0.9, 0.9], 'FaceAlpha', 0.3, 'EdgeColor', 'none');
legend('f(x) = -x^3', 'Boundaries at x = \pm1', 'Set K = [-1, 1]');

%% Plot the phase line diagram
subplot(2,1,2);
% Plot the x-axis
plot(x, zeros(size(x)), 'k');
hold on;

% Plot arrows indicating the direction of flow
for xi = linspace(x_min, x_max, 25)
    % Determine the direction
    direction = sign(f(xi));
    % Skip if at equilibrium point
    if direction == 0
        continue;
    end
    % Draw arrow
    quiver(xi, 0, 0.1*direction, 0, 'r', 'MaxHeadSize', 2, 'AutoScale', 'off');
end

% Mark the equilibrium point at x = 0
plot(0, 0, 'ko', 'MarkerFaceColor', 'k');

% Mark the boundaries of K
plot([-1, -1], [-0.1, 0.1], 'k--');
plot([1, 1], [-0.1, 0.1], 'k--');

xlabel('x');
ylabel('');
title('Phase Line Diagram');
ylim([-0.2, 0.2]);
xlim([x_min, x_max]);
grid on;
legend('Phase Line', 'Direction of Flow', 'Equilibrium at x=0', 'Boundaries at x = \pm1');

%% Simulate trajectories for various initial conditions
figure;
hold on;
% Time span for simulation
t_span = [0, 10];

% Initial conditions within K
x0_within = [-0.9, -0.5, 0, 0.5, 0.9];

% Initial conditions outside K
x0_outside = [-1.5, -1.1, 1.1, 1.5];

% Colors for plotting
colors_within = lines(length(x0_within));
colors_outside = lines(length(x0_outside));

% Simulate trajectories within K
for i = 1:length(x0_within)
    [t, x_t] = ode45(@(t,x) f(x), t_span, x0_within(i));
    plot(t, x_t, 'Color', colors_within(i,:), 'LineWidth', 2);
end

% Simulate trajectories outside K
for i = 1:length(x0_outside)
    [t, x_t] = ode45(@(t,x) f(x), t_span, x0_outside(i));
    plot(t, x_t, '--', 'Color', colors_outside(i,:), 'LineWidth', 2);
end

% Plot boundaries of K
plot([t_span(1), t_span(2)], [-1, -1], 'k--');
plot([t_span(1), t_span(2)], [1, 1], 'k--');

xlabel('Time t');
ylabel('State x(t)');
title('Trajectories of the System dx/dt = -x^3');
legend_entries = arrayfun(@(x0) sprintf('x_0 = %.1f', x0), [x0_within, x0_outside], 'UniformOutput', false);
legend(legend_entries{:}, 'Boundaries at x = \pm1');

grid on;

% Highlight the set K
ylim([x_min, x_max]);
fill([t_span(1), t_span(2), t_span(2), t_span(1)], [-1, -1, 1, 1], [0.9, 0.9, 0.9], 'FaceAlpha', 0.3, 'EdgeColor', 'none');

%% Conclusion text
disp('Simulation complete. Trajectories starting within K = [-1, 1] remain within K, illustrating forward invariance.');
