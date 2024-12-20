clear all
clc
close all

% Simple MDP with dynamic programming (Value Iteration)

% Number of states and actions
numStates = 3;
numActions = 2;

% Transition probabilities matrix P(s'|s, a)
P = zeros(numStates, numStates, numActions);
P(:,:,1) = [0.8 0.2 0.0; 0.1 0.7 0.2; 0.2 0.3 0.5]; % Transition probabilities for action 1
P(:,:,2) = [0.6 0.3 0.1; 0.4 0.5 0.1; 0.1 0.2 0.7]; % Transition probabilities for action 2

% Reward matrix R(s, a)
R = [5 1; 2 3; 4 2]; % Rewards for each state-action pair

% Discount factor
gamma = 0.9;

% Initialization
V = zeros(numStates, 1); % Initial value function
theta = 1e-3; % Convergence threshold

% Value iteration
while true
    delta = 0;
    for s = 1:numStates
        v = V(s);
        Q = zeros(1, numActions);
        for a = 1:numActions
            Q(a) = R(s, a) + gamma * sum(P(s,:,a) * V);
        end
        V(s) = max(Q);
        delta = max(delta, abs(v - V(s)));
    end
    
    % Check for convergence
    if delta < theta
        break;
    end
end

% Policy extraction
policy = zeros(numStates, 1);
for s = 1:numStates
    Q = zeros(1, numActions);
    for a = 1:numActions
        Q(a) = R(s, a) + gamma * sum(P(s,:,a) * V);
    end
    [~, policy(s)] = max(Q);
end

% Display results
disp('Optimal Value Function:');
disp(V);
disp('Optimal Policy:');
disp(policy);

% Illustration
figure;
subplot(1,2,1);
bar(V);
title('Optimal Value Function');
xlabel('States');
ylabel('Value');

subplot(1,2,2);
bar(policy);
title('Optimal Policy');
xlabel('States');
ylabel('Action');
