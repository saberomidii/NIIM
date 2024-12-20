% Policy Gradient Reinforcement Learning Example in MATLAB
% REINFORCE Algorithm in a Gridworld Environment

% Clear workspace and command window
clear; clc; close all;

%% 1. Define the Gridworld Environment

% Grid size
gridRows = 5;
gridCols = 5;

% Define the state space
numStates = gridRows * gridCols;

% Define actions: Up, Down, Left, Right
actions = {'U', 'D', 'L', 'R'};
numActions = length(actions);

% Define the reward structure
% -1 for each step, +10 for reaching the goal, -10 for hitting an obstacle
stepReward = -1;
goalReward = 10;
obstacleReward = -10;

% Define the goal state and obstacles
goalState = sub2ind([gridRows, gridCols], 5, 5); % Bottom-right corner
obstacles = [sub2ind([gridRows, gridCols], 3, 3)]; % Obstacle at (3,3)

% Transition function will be defined in the 'step' function

%% 2. Initialize Policy Parameters

% Policy parameters (theta) for each state-action pair
theta = zeros(numStates, numActions);

% Learning rate
alpha = 0.01;

% Discount factor
gamma = 0.99;

% Number of episodes
numEpisodes = 1000;

% Softmax function for policy
softmax = @(x) exp(x) ./ sum(exp(x));

%% 3. Define Helper Functions

% Function to get state index from coordinates
coord2state = @(row, col) sub2ind([gridRows, gridCols], row, col);

% Function to choose an action based on policy
function action = chooseAction(state, theta, actions)
    preferences = theta(state, :);
    probs = exp(preferences) / sum(exp(preferences));
    action = actions{randsample(1:length(actions), 1, true, probs)};
end

% Function to take a step in the environment
function [nextState, reward, done] = step(state, action, gridRows, gridCols, goalState, obstacles, stepReward, goalReward, obstacleReward)
    [row, col] = ind2sub([gridRows, gridCols], state);
    switch action
        case 'U'
            row = max(row - 1, 1);
        case 'D'
            row = min(row + 1, gridRows);
        case 'L'
            col = max(col - 1, 1);
        case 'R'
            col = min(col + 1, gridCols);
    end
    nextState = sub2ind([gridRows, gridCols], row, col);
    if ismember(nextState, obstacles)
        reward = obstacleReward;
        done = true;
    elseif nextState == goalState
        reward = goalReward;
        done = true;
    else
        reward = stepReward;
        done = false;
    end
end

%% 4. Training Loop

for episode = 1:numEpisodes
    % Initialize state
    state = coord2state(1, 1); % Start at top-left corner
    done = false;
    trajectory = [];
    rewards = [];
    states = [];
    actionsTaken = [];
    
    % Generate an episode
    while ~done
        % Choose action based on current policy
        action = chooseAction(state, theta, actions);
        
        % Take action, observe next state and reward
        [nextState, rewardReceived, done] = step(state, action, gridRows, gridCols, goalState, obstacles, stepReward, goalReward, obstacleReward);
        
        % Store the trajectory
        trajectory = [trajectory; state, find(strcmp(actions, action)), rewardReceived];
        rewards = [rewards; rewardReceived];
        states = [states; state];
        actionsTaken = [actionsTaken; find(strcmp(actions, action))];
        
        % Move to next state
        state = nextState;
    end
    
    % Compute returns (cumulative discounted rewards)
    G = zeros(size(rewards));
    Gt = 0;
    for t = length(rewards):-1:1
        Gt = rewards(t) + gamma * Gt;
        G(t) = Gt;
    end
    
    % Update policy parameters using REINFORCE
    for t = 1:length(states)
        s = states(t);
        a = actionsTaken(t);
        % Compute policy probabilities
        preferences = theta(s, :);
        probs = exp(preferences) / sum(exp(preferences));
        % Gradient ascent step
        grad = -probs;
        grad(a) = grad(a) + 1;
        theta(s, :) = theta(s, :) + alpha * gamma^(t-1) * G(t) * grad;
    end
    
    % Optionally, print progress
    if mod(episode, 100) == 0
        fprintf('Episode %d/%d completed.\n', episode, numEpisodes);
    end
end

%% 5. Testing the Learned Policy

% Display the learned policy
policy = cell(gridRows, gridCols);
for s = 1:numStates
    preferences = theta(s, :);
    [~, a_idx] = max(preferences);
    [row, col] = ind2sub([gridRows, gridCols], s);
    if s == goalState
        policy{row, col} = 'G'; % Goal
    elseif ismember(s, obstacles)
        policy{row, col} = 'X'; % Obstacle
    else
        policy{row, col} = actions{a_idx};
    end
end

% Display the policy grid
disp('Learned Policy:');
for i = 1:gridRows
    for j = 1:gridCols
        fprintf('%s\t', policy{i, j});
    end
    fprintf('\n');
end

%% 6. Visualizing an Episode with the Learned Policy

state = coord2state(1, 1); % Start at top-left corner
done = false;
trajectory = [state];
while ~done
    % Choose action based on learned policy
    preferences = theta(state, :);
    [~, a_idx] = max(preferences);
    action = actions{a_idx};
    
    % Take action
    [nextState, ~, done] = step(state, action, gridRows, gridCols, goalState, obstacles, stepReward, goalReward, obstacleReward);
    
    % Store the trajectory
    trajectory = [trajectory; nextState];
    
    % Move to next state
    state = nextState;
end

% Plot the trajectory
figure;
hold on;
% Plot grid lines
for i = 0:gridRows
    plot([0, gridCols], [i, i], 'k');
end
for j = 0:gridCols
    plot([j, j], [0, gridRows], 'k');
end
% Plot obstacles
for idx = obstacles
    [row, col] = ind2sub([gridRows, gridCols], idx);
    rectangle('Position', [col - 1, gridRows - row, 1, 1], 'FaceColor', 'k');
end
% Plot trajectory
for t = 1:length(trajectory)-1
    [row1, col1] = ind2sub([gridRows, gridCols], trajectory(t));
    [row2, col2] = ind2sub([gridRows, gridCols], trajectory(t+1));
    plot([col1 - 0.5, col2 - 0.5], [gridRows - row1 + 0.5, gridRows - row2 + 0.5], 'ro-', 'LineWidth', 2);
end
% Plot start and goal
[start_row, start_col] = ind2sub([gridRows, gridCols], trajectory(1));
[goal_row, goal_col] = ind2sub([gridRows, gridCols], goalState);
plot(start_col - 0.5, gridRows - start_row + 0.5, 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot(goal_col - 0.5, gridRows - goal_row + 0.5, 'bs', 'MarkerSize', 10, 'MarkerFaceColor', 'b');

xlim([0, gridCols]);
ylim([0, gridRows]);
xlabel('Grid Columns');
ylabel('Grid Rows');
title('Agent Trajectory using Learned Policy');
grid on;
set(gca, 'YDir', 'normal');
hold off;
