% Plot Experiment Data  2024-07-24_19-36 
%                       2024-07-24_19-46
%                       2024-07-24_19-56 
% Plot Experiment Data 
clear all
clc 
close all

m = 2;
n = 4;
Q = eye(n);
R = eye(m);

% Number of epochs
num_epochs = 15; % Adjust the number of epochs as needed

% Initialize arrays to store values for each epoch
cost_values = cell(num_epochs, 1);
Cost = zeros(num_epochs, 1);
Cost_p = zeros(num_epochs, 1);
last_time_step_cost = zeros(num_epochs, 1);
gain_matrices = cell(num_epochs, 1);
gain_norms = zeros(num_epochs, 1); % Array to store the norm of the gain matrices
trained_gains = cell(num_epochs, 1);
P_matrices = cell(num_epochs, 1);
P_norms = zeros(num_epochs, 1); % Array to store the norm of the solution matrices
trained_P_matrices = cell(num_epochs, 1); % Cell array to store the trained P matrices
state_1_values = cell(num_epochs, 1); % Cell array to store state 1 values for each epoch
state_2_values = cell(num_epochs, 1); % Cell array to store state 2 values for each epoch
state_3_values = cell(num_epochs, 1); % Cell array to store state 3 values for each epoch
state_4_values = cell(num_epochs, 1); % Cell array to store state 4 values for each epoch
input_1_values = cell(num_epochs, 1); % Cell array to store input 1 values for each epoch
input_2_values = cell(num_epochs, 1); % Cell array to store input 2 values for each epoch

% Loop over each epoch to load data, calculate the cost function, and store the cost values
for epoch = 1:num_epochs
    % Load Data for current epoch
    data_path = sprintf("C:\\Users\\saber\\OneDrive\\Desktop\\Research\\RL LQR\\Magnetic Levitation\\Experiment\\Data DPI\\2024-07-24_19-46\\epoch %d\\workspace.mat", epoch);
    Data = load(data_path);
    
    % Extract matrices X and U from loaded data
    % Adjust according to your data structure
    X = Data.x_star(:,2:end)'; % Example extraction
    U = Data.u_star(:,2:end)'; % Example extraction

    % Extract the gain matrix for the current epoch
    gain_matrix = Data.K; % Assuming gain_matrix is the name in the data
    gain_matrices{epoch} = gain_matrix;
    gain_norms(epoch) = norm(gain_matrix, 'fro'); % Calculate the Frobenius norm

    % Extract the trained gains for the current epoch
    trained_gains{epoch} = Data.err_K; % Assuming trained_gains is the name in the data
    
    % Extract the solution matrix P for the current epoch
    P_matrix = Data.P; % Assuming P is the name in the data
    P_matrices{epoch} = P_matrix;
    P_norms(epoch) = norm(P_matrix, 'fro'); % Calculate the Frobenius norm
    
    % Extract the trained P matrix for the current epoch
    trained_P_matrices{epoch} = Data.err_P; % Assuming trained_P is the name in the data
    
    % Store state and input values for the current epoch
    state_1_values{epoch} = X(1, :);
    state_2_values{epoch} = X(2, :);
    state_3_values{epoch} = X(3, :);
    state_4_values{epoch} = X(4, :);
    input_1_values{epoch} = U(1, :);
    input_2_values{epoch} = U(2, :);
    
    % Number of time steps
    num_time_steps = size(X, 2);

    % Initialize array to store cost values for each time step
    cost_values_epoch = zeros(1, num_time_steps);

    % Calculate the quadratic cost function for each time step
    for t = 1:num_time_steps
        x_t = X(:, t); % Extract the state vector at time step t
        u_t = U(:, t); % Extract the control vector at time step t
        J_t = x_t' * Q * x_t + u_t' * R * u_t; % Calculate the cost at time step t
        cost_values_epoch(t) = J_t; % Store the cost value
    end
    
    % Store the cost values for the current epoch
    cost_values{epoch} = cost_values_epoch;
    
    % Calculate the average cost for the current epoch
    Cost(epoch) = sum(cost_values_epoch);
    Cost_p(epoch)=X(:,1)'*P_matrices{epoch}*X(:,1);
end


% Plot the average cost for each epoch
figure;
hold on;
plot(Cost, 'b', 'LineWidth', 1.5); % Blue line for \sigma x^TQX + U^T*R*U
plot(Cost_p, 'r-', 'LineWidth', 1.5); % Red line for X^T_0P_iX

% Axis labels with LaTeX
xlabel('Number of Epochs', 'Interpreter', 'latex');
ylabel('Value of Cost Function', 'Interpreter', 'latex');

% Title (if needed, currently left empty)
title('', 'Interpreter', 'latex');

% Grid
grid on;

% Legend with LaTeX
legend({'$\Sigma_{i=1}^T x^T Q x + u^T R u$', '$X^T_0 P_{epoch} X$'}, 'Interpreter', 'latex', 'Location', 'best');
