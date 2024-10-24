% Clear workspace and command window
clear; clc; close all;

%% System Parameters
a = 1.1;       % System parameter (unstable system since |a| > 1)
b = 0.5;       % Control input coefficient
sigma_w = 0.1; % Standard deviation of process noise

%% Discretization Parameters
x_min = -10; x_max = 10; dx = 0.5;
u_min = -10; u_max = 10; du = 0.5;

x_grid = x_min:dx:x_max;   % Discretized state space
u_grid = u_min:du:u_max;   % Discretized action space

num_states = length(x_grid);
num_actions = length(u_grid);

%% Q-learning Parameters
Q = zeros(num_states, num_actions);  % Initialize Q-table

alpha = 0.1;      % Learning rate
gamma = 0.99;     % Discount factor
epsilon = 0.1;    % Exploration rate

num_episodes = 1000;   % Number of episodes
max_steps = 50;        % Max steps per episode

%% Reward Function
% Reward is defined as negative squared state to minimize state magnitude
reward_func = @(x) -x^2;

%% Storage for plotting
episode_rewards = zeros(num_episodes,1);

%% Q-learning Algorithm
for episode = 1:num_episodes
    % Initialize state x_0 randomly within [-10, 10]
    x = (x_max - x_min) * rand() + x_min;
    
    total_reward = 0;
    
    for step = 1:max_steps
        % Discretize current state
        [~, s_idx] = min(abs(x_grid - x));
        
        % Epsilon-greedy action selection
        if rand() < epsilon
            % Explore: select a random action
            a_idx = randi(num_actions);
        else
            % Exploit: select action with highest Q-value
            [~, a_idx] = max(Q(s_idx, :));
        end
        
        % Map action index to control input
        u = u_grid(a_idx);
        
        % Apply control input to the system with process noise
        w = sigma_w * randn();  % Process noise
        x_next = a * x + b * u + w;
        
        % Receive reward
        r = reward_func(x);
        total_reward = total_reward + r;
        
        % Discretize next state
        x_next_clipped = min(max(x_next, x_min), x_max);  % Clip to state bounds
        [~, s_next_idx] = min(abs(x_grid - x_next_clipped));
        
        % Q-learning update
        best_next_Q = max(Q(s_next_idx, :));
        Q(s_idx, a_idx) = Q(s_idx, a_idx) + alpha * (r + gamma * best_next_Q - Q(s_idx, a_idx));
        
        % Update state
        x = x_next;
        
        % Check for stability (terminate if state diverges)
        if abs(x) > 1e6
            break;
        end
    end
    
    % Store total reward for this episode
    episode_rewards(episode) = total_reward;
    
    % Optionally decrease epsilon over time (epsilon decay)
    % epsilon = max(epsilon * 0.995, 0.01);
end

%% Policy Extraction
% After learning, extract the optimal policy from Q-table
[~, optimal_action_indices] = max(Q, [], 2);
optimal_policy = u_grid(optimal_action_indices);

%% Plotting Results

% Plot the learning curve (total reward per episode)
figure;
plot(1:num_episodes, episode_rewards);
xlabel('Episode');
ylabel('Total Reward');
title('Learning Curve');

% Plot the optimal policy
figure;
plot(x_grid, optimal_policy);
xlabel('State x');
ylabel('Optimal Control Input u');
title('Optimal Policy');

%% Stability Analysis
% Check if the closed-loop system is stable under the optimal policy
% The closed-loop system is x_{k+1} = (a + b*K)x_k
% We need |a + b*K| < 1 for stability

% Compute the effective gain K for each state
K_effective = optimal_policy ./ x_grid';

% Since x can be zero, handle division by zero
K_effective(isnan(K_effective) | isinf(K_effective)) = 0;

% Compute the closed-loop eigenvalues
closed_loop_eigenvalues = a + b * K_effective;

% Plot the closed-loop eigenvalues
figure;
plot(x_grid, closed_loop_eigenvalues);
hold on;
plot(x_grid, ones(size(x_grid)), '--r');
plot(x_grid, -ones(size(x_grid)), '--r');
xlabel('State x');
ylabel('Closed-loop Eigenvalue \lambda');
title('Closed-loop Eigenvalues');
legend('\lambda = a + bK', '\lambda = 1', '\lambda = -1');

% Display stability regions
stable_indices = abs(closed_loop_eigenvalues) < 1;
fprintf('Percentage of states with stable closed-loop dynamics: %.2f%%\n', ...
    100 * sum(stable_indices) / num_states);

%% Simulate the System with Optimal Policy
% Starting from different initial conditions

initial_conditions = [-8, -4, 0.1, 4, 8];
num_initial_conditions = length(initial_conditions);
simulation_steps = 30;

figure;
hold on;

for idx = 1:num_initial_conditions
    x = initial_conditions(idx);
    x_trajectory = zeros(simulation_steps,1);
    
    for k = 1:simulation_steps
        x_trajectory(k) = x;
        
        % Discretize current state
        [~, s_idx] = min(abs(x_grid - x));
        
        % Get control input from optimal policy
        u = optimal_policy(s_idx);
        
        % Apply control input to the system without noise
        x = a * x + b * u;
    end
    
    plot(1:simulation_steps, x_trajectory, 'LineWidth', 2);
end

xlabel('Time Step');
ylabel('State x');
title('State Trajectories under Optimal Policy');
legend(arrayfun(@(x0) sprintf('x_0 = %.1f', x0), initial_conditions, 'UniformOutput', false));

hold off;
