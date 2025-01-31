function [uOpt_value, V_value] = computeValueFunction(x, y, theta, time)
    % Add paths to helperOC-master and Level Set Methods toolbox
    addpath(genpath('C:\Users\saber\OneDrive\Desktop\Research\NIIM\weekly-meeting\29 July 2024\helperOC-master'));
    addpath(genpath('C:\Users\saber\OneDrive\Desktop\Research\NIIM\weekly-meeting\29 July 2024\ToolboxLS'));

    %% Grid setup
    grid_min = [-5; -5; -pi];   % Lower bounds [x; y; theta]
    grid_max = [5; 5; pi];      % Upper bounds [x; y; theta]
    N = [100; 100; 50];         % Number of grid points in each dimension

    g = createGrid(grid_min, grid_max, N);

    %%% Define the feasible set 
    x_min = 0; % Minimum x-coordinate of the rectangle
    x_max = 2;  % Maximum x-coordinate of the rectangle
    theta_min = -0.5; % Minimum theta
    theta_max = 0.5;  % Maximum theta

    % Define the desired trajectory and offset functions
    desired_trajectory = @(x) 0.75 * x.^2; % Scaled quadratic trajectory
    offset_positive = @(x) desired_trajectory(x) + 0.2;
    offset_negative = @(x) desired_trajectory(x) - 0.2;

    % Create a matrix to store whether each point is within the feasible set
    in_feasible_set = zeros(N(1), N(2), N(3));

    % Iterate through each point in the grid and check if it is within the feasible set
    for i = 1:N(1)
        for j = 1:N(2)
            for k = 1:N(3)
                xg = g.xs{1}(i, j, k);
                yg = g.xs{2}(i, j, k);
                thetag = g.xs{3}(i, j, k);

                % Compute y bounds based on the desired trajectory
                if xg >= x_min && xg <= x_max
                    y_min = offset_negative(xg);
                    y_max = offset_positive(xg);
                else
                    y_min = -inf; % No bounds if x is outside [x_min, x_max]
                    y_max = inf;
                end

                % Check if the point (x, y, theta) is within the feasible set
                if (xg >= x_min && xg <= x_max) && (yg >= y_min && yg <= y_max) && (thetag >= theta_min && thetag <= theta_max)
                    in_feasible_set(i, j, k) = -min(vecnorm([xg; yg; thetag] - [x; y; theta]));
                else
                    in_feasible_set(i, j, k) = min(vecnorm([xg; yg; thetag] - [x; y; theta]));
                end
            end
        end
    end

    l = -in_feasible_set;
    data0 = l; % Use l as the initial value function directly

    % Dubins car dynamics
    speed = 0.2;
    wMax = 1;
    dCar = DubinsCar([], wMax, speed);

    %% Time vector
    t0 = 0;
    tMax = time;
    dt = 1;
    tau = t0:dt:tMax;

    %% Scheme data
    schemeData.grid = g;
    schemeData.dynSys = dCar;
    schemeData.accuracy = 'high'; % Options: 'low', 'medium', 'high'
    schemeData.uMode = 'max';    % Minimize the cost function

    %% Solve the HJB PDE with target function
    HJIextraArgs.visualize = false;                % Disable visualization for function
    HJIextraArgs.targetFunction = l;              % Target function

    [V, Time_vector] = HJIPDE_solve(data0, tau, schemeData, 'minVWithTarget', HJIextraArgs);

    % Compute the gradient of V with respect to theta
    dV_dtheta = gradient(V, g.dx(3));
    wRange = [-1 1];

    % Determine the optimal control
    uOpt = (dV_dtheta >= 0) * wRange(2) + (dV_dtheta < 0) * wRange(1);

    %% Interpolate to find the value function and optimal control at (x, y, theta, time)
    % Define the time grid points
    time_points = linspace(t0, tMax, length(Time_vector));

    % Interpolate the value function at (x, y, theta, time)
    V_value = interpn(g.xs{1}, g.xs{2}, g.xs{3}, V, x, y, theta);
    V_value=V_value(:,:,:,end);
    
    % Interpolate the optimal control at (x, y, theta, time)
    uOpt_value = interpn(g.xs{1}, g.xs{2}, g.xs{3}, uOpt, x, y, theta);
    uOpt_value=  uOpt_value(:,:,:,end);
end
