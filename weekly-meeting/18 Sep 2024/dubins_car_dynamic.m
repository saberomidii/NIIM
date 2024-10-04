function next_state = dubins_car_dynamic(state, input, dt, v)
    % DUBINS_CAR Computes the next state of a Dubins car.
    %
    %   next_state = DUBINS_CAR(state, input, dt, v) returns the next state
    %   of the Dubins car given the current state, steering input, time step,
    %   and constant speed.
    %
    %   Parameters:
    %       state - Current state vector [x; y; theta]
    %       input - Steering input (turning rate) u
    %       dt    - Time step
    %       v     - Constant forward speed
    %
    %   Returns:
    %       next_state - Next state vector [x_next; y_next; theta_next]

    % Extract current state variables
    x = state(1);
    y = state(2);
    theta = state(3);

    % Steering input
    u = input;

    % Compute next state using Dubins car dynamics
    x_next = x + v * cos(theta) * dt;
    y_next = y + v * sin(theta) * dt;
    theta_next = theta + u * dt;

    % Normalize theta to be within [-pi, pi]
    theta_next = atan2(sin(theta_next), cos(theta_next));

    % Return next state
    next_state = [x_next; y_next; theta_next];
end
