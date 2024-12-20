% Two-Player Game: Agent vs. Adversary in a Circular Set using Policy Gradient Reinforcement Learning

% Clear workspace and command window
clear; clc; close all;

%% 1. Define the Environment and Dynamics

% Circular set: Center at (0,0) and radius 5
set_center = [0; 0];
set_radius = 5;

% Define the state space
% State: [x; y; theta]

% Dubins car parameters
v = 1.0;  % Constant speed
dt = 0.1; % Time step

% Discretize the agent's action space (steering angles)
numAgentActions = 50;
u_limits = [-1, 1];  % Agent's steering angle limits (radians)
u_values = linspace(u_limits(1), u_limits(2), numAgentActions);

% Discretize the adversary's action space (disturbances)
numAdvActions = 5;
d_limits = [-0.5, 0.5];  % Adversary's disturbance limits (radians)
d_values = linspace(d_limits(1), d_limits(2), numAdvActions);

% Number of episodes
numEpisodes = 1000;

% Learning rate
alpha = 0.01;

% Discount factor
gamma = 0.99;

% Policy parameters (theta) for a function approximator
% For simplicity, we use a grid-based representation
gridSize = 20;  % Number of grid points along each dimension
x_grid = linspace(set_center(1) - set_radius, set_center(1) + set_radius, gridSize);
y_grid = linspace(set_center(2) - set_radius, set_center(2) + set_radius, gridSize);
theta_grid = linspace(-pi, pi, gridSize);

% Total number of states (discretized for function approximation)
numStates = gridSize^3;

% Initialize policy parameters (theta) randomly
theta = zeros(numStates, numAgentActions);

% Softmax function for policy
softmax = @(x) exp(x) ./ sum(exp(x));

%% 2. Define Helper Functions

% Function to discretize the state
function state_idx = discretizeState(state, x_grid, y_grid, theta_grid)
    [~, x_idx] = min(abs(x_grid - state(1)));
    [~, y_idx] = min(abs(y_grid - state(2)));
    [~, theta_idx] = min(abs(theta_grid - state(3)));
    state_idx = sub2ind([length(x_grid), length(y_grid), length(theta_grid)], x_idx, y_idx, theta_idx);
end

% Function to choose an action based on policy
function action = chooseAgentAction(state_idx, theta, u_values)
    preferences = theta(state_idx, :);
    probs = exp(preferences) / sum(exp(preferences));
    action_idx = randsample(1:length(u_values), 1, true, probs);
    action = u_values(action_idx);
end

% Function to choose adversary's action
function action = chooseAdversaryAction(state, d_values, set_center)
    % Adversary policy: Push the agent away from the set center
    % Compute the direction away from the center
    dir_to_center = atan2(set_center(2) - state(2), set_center(1) - state(1));
    dir_away = wrapToPi(dir_to_center + pi);  % Opposite direction
    % Compute desired heading change to move away
    desired_heading_change = wrapToPi(dir_away - state(3));
    % Choose disturbance that aligns with desired heading change
    [~, idx] = min(abs(d_values - desired_heading_change / dt));
    action = d_values(idx);
end

% Function to compute reward
function reward = computeReward(state, set_center, set_radius)
    % Check if the state is within the set
    dist_to_center = norm(state(1:2) - set_center);
    if dist_to_center > set_radius
        reward = -100;  % Negative reward for going outside
    else
        % Positive reward for staying inside
        reward = 1;  % Reward per time step inside the set
    end
end

% Function to update the state based on dynamics
function next_state = dynamics(state, u_agent, u_adv, v, dt)
    % Dubins car dynamics with adversary input
    x = state(1) + v * cos(state(3)) * dt;
    y = state(2) + v * sin(state(3)) * dt;
    theta = state(3) + (u_agent + u_adv) * dt;  % Agent and adversary inputs add up
    % Wrap theta between -pi and pi
    theta = wrapToPi(theta);
    next_state = [x; y; theta];
end

%% 3. Training Loop

for episode = 1:numEpisodes
    % Initialize state within the set (e.g., center of the set)
    state = [set_center; 0];  % Starting at the center facing right
    done = false;
    trajectory = [];
    rewards = [];
    states = [];
    actionsTaken = [];
    state_idxs = [];
    action_idxs = [];
    
    maxSteps = 300;  % To represent 30 seconds (dt = 0.1)
    step = 1;
    while ~done && step <= maxSteps
        % Discretize the state for function approximation
        state_idx = discretizeState(state, x_grid, y_grid, theta_grid);
        
        % Agent chooses action based on current policy
        u_agent = chooseAgentAction(state_idx, theta, u_values);
        agent_action_idx = find(u_values == u_agent);
        
        % Adversary chooses action
        u_adv = chooseAdversaryAction(state, d_values, set_center);
        
        % Take action, observe next state and reward
        next_state = dynamics(state, u_agent, u_adv, v, dt);
        rewardReceived = computeReward(next_state, set_center, set_radius);
        
        % Check if the episode is done
        if rewardReceived <= -100  % Went outside the set
            done = true;
        end
        
        % Store the trajectory
        trajectory = [trajectory; state', u_agent, u_adv, rewardReceived];
        rewards = [rewards; rewardReceived];
        states = [states, state];
        actionsTaken = [actionsTaken; u_agent];
        state_idxs = [state_idxs; state_idx];
        action_idxs = [action_idxs; agent_action_idx];
        
        % Move to next state
        state = next_state;
        step = step + 1;
    end
    
    % Compute returns (cumulative discounted rewards)
    G = zeros(size(rewards));
    Gt = 0;
    for t = length(rewards):-1:1
        Gt = rewards(t) + gamma * Gt;
        G(t) = Gt;
    end
    
    % Update policy parameters using REINFORCE
    for t = 1:length(state_idxs)
        s_idx = state_idxs(t);
        a_idx = action_idxs(t);
        % Compute policy probabilities
        preferences = theta(s_idx, :);
        probs = exp(preferences) / sum(exp(preferences));
        % Gradient ascent step
        grad = -probs;
        grad(a_idx) = grad(a_idx) + 1;
        theta(s_idx, :) = theta(s_idx, :) + alpha * gamma^(t-1) * G(t) * grad;
    end
    
    % Optionally, print progress
    if mod(episode, 100) == 0
        fprintf('Episode %d/%d completed. Time inside set: %.1f seconds\n', episode, numEpisodes, min(step * dt, maxSteps * dt));
    end
end

%% 4. Testing the Learned Policy

% Simulate the agent using the learned policy against the adversary
state = [set_center; 0];  % Starting at the center facing right
done = false;
trajectory = [state'];
step = 1;
while ~done && step <= maxSteps
    % Discretize the state
    state_idx = discretizeState(state, x_grid, y_grid, theta_grid);
    
    % Agent chooses action based on learned policy
    preferences = theta(state_idx, :);
    [~, a_idx] = max(preferences);
    u_agent = u_values(a_idx);
    
    % Adversary chooses action
    u_adv = chooseAdversaryAction(state, d_values, set_center);
    
    % Take action
    next_state = dynamics(state, u_agent, u_adv, v, dt);
    rewardReceived = computeReward(next_state, set_center, set_radius);
    
    % Check if the episode is done
    if rewardReceived <= -100  % Went outside the set
        done = true;
    end
    
    % Store the trajectory
    trajectory = [trajectory; next_state'];
    
    % Move to next state
    state = next_state;
    step = step + 1;
end

fprintf('Test run: Time inside set: %.1f seconds\n', min(step * dt, maxSteps * dt));

%% 5. Visualizing the Trajectory

% Plot the circular set
theta_circle = linspace(0, 2*pi, 100);
circle_x = set_center(1) + set_radius * cos(theta_circle);
circle_y = set_center(2) + set_radius * sin(theta_circle);

figure;
hold on;
plot(circle_x, circle_y, 'k-', 'LineWidth', 2);  % Boundary of the set

% Plot the trajectory
plot(trajectory(:,1), trajectory(:,2), 'b-', 'LineWidth', 2);

% Plot starting point
plot(trajectory(1,1), trajectory(1,2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');

% Plot ending point
plot(trajectory(end,1), trajectory(end,2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');

xlabel('x');
ylabel('y');
title('Dubins Car Trajectory with Active Adversary');
axis equal;
grid on;
hold off;
