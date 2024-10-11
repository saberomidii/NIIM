% Define the system of differential equations with control parameter u
function dydt = system_eqs(t, y, u)
    x = y(1);
    y_var = y(2); % Using 'y_var' to avoid conflict with MATLAB function 'y'
    
    % Define the dynamics with control input u
    dxdt = x - y_var - x*(x^2 + y_var^2);
    dydt = x + y_var - y_var*(x^2 + y_var^2) + u;
    
    dydt = [dxdt; dydt]; % Return derivatives as a column vector
end

% Time span for the simulation
tspan = [0 30]; % You can adjust the time span as needed

% Initial conditions [x(0), y(0)]
y0 = [0.3; 0.3]; % You can adjust the initial conditions as needed

% Define the control input u
u = 0; % You can change the value of u to see different effects on the system

% Solve the system of ODEs using ode45 and pass u as an additional parameter
[t, y] = ode45(@(t, y) system_eqs(t, y, u), tspan, y0);

% Create a grid for the vector field
[X, Y] = meshgrid(-2:0.2:2, -2:0.2:2); % Define a grid in the range [-2, 2]
U = zeros(size(X)); % Initialize for dx/dt values
V = zeros(size(Y)); % Initialize for dy/dt values

% Compute the vector field at each point in the grid with control u
for i = 1:numel(X)
    x = X(i);
    y_var = Y(i);
    U(i) = x - y_var - x*(x^2 + y_var^2); % dx/dt (no control for this equation)
    V(i) = x + y_var - y_var*(x^2 + y_var^2) + u; % dy/dt with control u
end

% Plot the vector field
figure;
quiver(X, Y, U, V, 'r'); % Plot the vector field with red arrows
hold on;

% Plot the trajectory of x and y
plot(y(:,1), y(:,2), '-b', 'LineWidth', 2); % Plot trajectory in blue
plot(y(1,1), y(1,2), 'go', 'MarkerSize', 8, 'DisplayName', 'Start'); % Mark the start point
plot(y(end,1), y(end,2), 'ro', 'MarkerSize', 8, 'DisplayName', 'End'); % Mark the end point
xlabel('x');
ylabel('y');
title('Vector Field and Trajectory of the System with Control');
axis([-2 2 -2 2]); % Set axis limits for the plot
grid on;
hold off;

% Plot the results over time separately
figure;
plot(t, y(:,1), '-o', 'DisplayName', 'x(t)');
hold on;
plot(t, y(:,2), '-s', 'DisplayName', 'y(t)');
xlabel('Time t');
ylabel('Solution');
legend;
title('Solution of the Differential Equations with Control');
grid on;
