% % MATLAB Code: Simulating Movement to a Neighbor Node and Plotting Two PDFs
% 
% % --------------------- Initial Setup ---------------------
% 
% % Define the coordinates of the 9 nodes in a 3x3 rectangular grid
% [x1_grid, x2_grid] = meshgrid(-1:1, -1:1);  % x1 and x2 range from -1 to 1
% 
% % Assign z-values as zeros
% z_grid = zeros(size(x1_grid));
% 
% % Flatten the grids for plotting
% x1 = x1_grid(:);
% x2 = x2_grid(:);
% z = z_grid(:);
% 
% % Plot the nodes in 3D
% figure;
% set(gcf, 'Color', [0.1, 0.1, 0.1]);  % Set figure background to dark gray
% scatter3(x1, x2, z, 100, 'filled', 'MarkerFaceColor', 'b');
% hold on;
% 
% % --------------------- First Starting Point at (0, 0) ---------------------
% 
% % Highlight the origin node at (0, 0)
% idx_origin = find(x1 == 0 & x2 == 0);
% scatter3(x1(idx_origin), x2(idx_origin), z(idx_origin), 150, 'filled', 'MarkerFaceColor', 'g');
% 
% % Define negative probability for the origin node (between -1 and 0)
% prob_origin = -rand();  % Negative random probability between -1 and 0
% disp(['Negative probability at origin node (0, 0): ', num2str(prob_origin)]);
% 
% % Adjust the marker size based on the absolute value of prob_origin
% origin_marker_size = 100 + 100 * abs(prob_origin);
% scatter3(x1(idx_origin), x2(idx_origin), z(idx_origin), origin_marker_size, 'filled', 'MarkerFaceColor', 'g');
% 
% % Display the negative probability at the origin node
% text(x1(idx_origin), x2(idx_origin), z(idx_origin) + 0.1, sprintf('P=%.2f', prob_origin), ...
%     'FontSize', 10, 'Color', 'w', 'EdgeColor', 'k', 'HorizontalAlignment', 'center');
% 
% % Find the indices of neighboring nodes for the origin node
% neighbor_indices = [];
% for idx = 1:length(x1)
%     if idx ~= idx_origin
%         % Calculate the distance from the origin node
%         distance = sqrt((x1(idx) - x1(idx_origin))^2 + (x2(idx) - x2(idx_origin))^2);
%         if distance == 1 || distance == sqrt(2)  % Immediate neighbors
%             neighbor_indices = [neighbor_indices; idx];
%         end
%     end
% end
% 
% % Connect the origin node to its neighboring nodes with red lines
% prob_lines = zeros(length(neighbor_indices), 1);  % To store negative probabilities for each line
% 
% for idx = 1:length(neighbor_indices)
%     neighbor_idx = neighbor_indices(idx);
%     % Assign a negative random probability to the line (between -1 and 0)
%     prob_line = -rand();  % Negative random probability between -1 and 0
%     prob_lines(idx) = prob_line;
% 
%     x_coords = [x1(idx_origin), x1(neighbor_idx)];
%     y_coords = [x2(idx_origin), x2(neighbor_idx)];
%     z_coords = [z(idx_origin), z(neighbor_idx)];
% 
%     % Adjust line width based on the absolute value of probability
%     line_width = 1 + 4 * abs(prob_line);  % Line width between 1 and 5
% 
%     % Plot the line with adjusted line width
%     plot3(x_coords, y_coords, z_coords, 'r-', 'LineWidth', line_width);
% 
%     % Display the negative probability as white text with black edge
%     mid_x = mean(x_coords);
%     mid_y = mean(y_coords);
%     mid_z = mean(z_coords);
%     text(mid_x, mid_y, mid_z, sprintf('%.2f', prob_line), 'FontSize', 8, ...
%         'Color', 'w', 'EdgeColor', 'k', 'HorizontalAlignment', 'center');
% end
% 
% % Calculate the average negative probability of the neighboring lines
% average_probability = mean(prob_lines);
% disp(['Average negative probability of the neighboring lines at (0, 0): ', num2str(average_probability)]);
% 
% % Adjust sigma_normal so that the normal distribution is effectively zero at neighboring nodes
% epsilon = 1e-6;  % Tolerance for considering the value as zero
% d_min = 1;       % Minimum distance to neighboring nodes
% 
% % Calculate sigma_normal
% sigma_normal = sqrt( - (d_min^2) / (2 * log(epsilon)) );
% 
% % Define the normal distribution centered at the origin with adjusted sigma_normal
% [x1_normal, x2_normal] = meshgrid(-1:0.01:1, -1:0.01:1);  % Grid covering neighborhood
% z_normal = average_probability * exp(- ((x1_normal - x1(idx_origin)).^2 + (x2_normal - x2(idx_origin)).^2) / (2 * sigma_normal^2));
% 
% % Plot the normal distribution surface for the first starting point
% surf(x1_normal, x2_normal, z_normal, 'EdgeColor', 'none', 'FaceAlpha', 0.5);
% 
% % --------------------- Move to Neighboring Node at (0, 1) ---------------------
% 
% % Define the new starting point (moving "up" to (0, 1))
% new_start_x = 0;
% new_start_y = 1;
% idx_new_origin = find(x1 == new_start_x & x2 == new_start_y);
% 
% % Highlight the new origin node
% scatter3(x1(idx_new_origin), x2(idx_new_origin), z(idx_new_origin), 150, 'filled', 'MarkerFaceColor', 'c');
% 
% % Define negative probability for the new origin node (between -1 and 0)
% prob_new_origin = -rand();  % Negative random probability between -1 and 0
% disp(['Negative probability at new origin node (0, 1): ', num2str(prob_new_origin)]);
% 
% % Adjust the marker size based on the absolute value of prob_new_origin
% new_origin_marker_size = 100 + 100 * abs(prob_new_origin);
% scatter3(x1(idx_new_origin), x2(idx_new_origin), z(idx_new_origin), new_origin_marker_size, 'filled', 'MarkerFaceColor', 'c');
% 
% % Display the negative probability at the new origin node
% text(x1(idx_new_origin), x2(idx_new_origin), z(idx_new_origin) + 0.1, sprintf('P=%.2f', prob_new_origin), ...
%     'FontSize', 10, 'Color', 'w', 'EdgeColor', 'k', 'HorizontalAlignment', 'center');
% 
% % Find the indices of neighboring nodes for the new origin node
% new_neighbor_indices = [];
% for idx = 1:length(x1)
%     if idx ~= idx_new_origin
%         % Calculate the distance from the new origin node
%         distance = sqrt((x1(idx) - x1(idx_new_origin)).^2 + (x2(idx) - x2(idx_new_origin)).^2);
%         if distance == 1 || distance == sqrt(2)  % Immediate neighbors
%             new_neighbor_indices = [new_neighbor_indices; idx];
%         end
%     end
% end
% 
% % Connect the new origin node to its neighboring nodes with magenta lines
% prob_new_lines = zeros(length(new_neighbor_indices), 1);  % To store negative probabilities for each line
% 
% for idx = 1:length(new_neighbor_indices)
%     neighbor_idx = new_neighbor_indices(idx);
%     % Assign a negative random probability to the line (between -1 and 0)
%     prob_line = -rand();  % Negative random probability between -1 and 0
%     prob_new_lines(idx) = prob_line;
% 
%     x_coords = [x1(idx_new_origin), x1(neighbor_idx)];
%     y_coords = [x2(idx_new_origin), x2(neighbor_idx)];
%     z_coords = [z(idx_new_origin), z(neighbor_idx)];
% 
%     % Adjust line width based on the absolute value of probability
%     line_width = 1 + 4 * abs(prob_line);  % Line width between 1 and 5
% 
%     % Plot the line with adjusted line width
%     plot3(x_coords, y_coords, z_coords, 'm-', 'LineWidth', line_width);  % Magenta lines
% 
%     % Display the negative probability as white text with black edge
%     mid_x = mean(x_coords);
%     mid_y = mean(y_coords);
%     mid_z = mean(z_coords);
%     text(mid_x, mid_y, mid_z, sprintf('%.2f', prob_line), 'FontSize', 8, ...
%         'Color', 'w', 'EdgeColor', 'k', 'HorizontalAlignment', 'center');
% end
% 
% % Calculate the average negative probability of the neighboring lines for the new origin
% average_new_probability = mean(prob_new_lines);
% disp(['Average negative probability of the neighboring lines at (0, 1): ', num2str(average_new_probability)]);
% 
% % Define the normal distribution centered at the new origin with adjusted sigma_normal
% z_new_normal = average_new_probability * exp(- ((x1_normal - x1(idx_new_origin)).^2 + (x2_normal - x2(idx_new_origin)).^2) / (2 * sigma_normal^2));
% 
% % Plot the normal distribution surface for the new starting point
% surf(x1_normal, x2_normal, z_new_normal, 'EdgeColor', 'none', 'FaceAlpha', 0.5);
% 
% % --------------------- Final Adjustments ---------------------
% 
% % Adjust colormap to accommodate negative values
% colormap('jet');
% colorbar;
% 
% % Adjust the z-axis limits to include negative values
% min_z = min([average_probability, average_new_probability]) * 1.1;
% zlim([min_z, 0.1]);  % Since probabilities are negative
% 
% % Adjust the viewing angle and limits
% view(45, 30);
% xlim([-1.5, 1.5]);
% ylim([-1.5, 1.5]);
% hold off;
% 
% % --------------------- Additional Plot: Side View of Both Normal Distributions ---------------------
% 
% % Create a new figure for the side view
% figure;
% set(gcf, 'Color', [1, 1, 1]);  % Set figure background to white
% 
% % Plot the first normal distribution surface
% surf(x1_normal, x2_normal, z_normal, 'EdgeColor', 'none', 'FaceAlpha', 0.7);
% hold on;
% 
% % Plot the second normal distribution surface
% surf(x1_normal, x2_normal, z_new_normal, 'EdgeColor', 'none', 'FaceAlpha', 0.7);
% 
% % Customize the colormap and remove axes for a clean look
% colormap('jet');
% colorbar;
% shading interp;  % Smooth color transition
% 
% % Set labels and title
% xlabel('x1');
% ylabel('x2');
% zlabel('Probability Density');
% title('Normal Distributions (PDFs) Side View at (0, 0) and (0, 1)');
% 
% % Adjust the viewing angle to a side view
% view(0, 0);  % Side view along the x-axis
% 
% % Adjust axis limits to include negative probabilities
% xlim([-1.5, 1.5]);
% ylim([-1.5, 1.5]);
% zlim([min_z, 0.1]);  % Adjust z-axis for negative values
% 
% % Add grid lines
% grid on;
% 
% % Add legend to distinguish the two PDFs
% legend({'PDF at (0, 0)', 'PDF at (0, 1)'}, 'Location', 'northeast');
% 
% hold off;


% MATLAB Code: 10x10 Grid with 3 Kernels and Edge Handling

% MATLAB Code: Define a 10x10 Grid of Nodes

% MATLAB Code: 10x10 Grid of Nodes with Connections to Neighbors

% MATLAB Code: 10x10 Grid with Connections and Random Probabilities

% % Clear workspace and figures
% clear;
% close all;
% 
% % Define the grid coordinates
% [x1_grid, x2_grid] = meshgrid(0:9, 0:9);
% 
% % Flatten the grids for plotting
% x1 = x1_grid(:);  % x-coordinates of nodes
% x2 = x2_grid(:);  % y-coordinates of nodes
% z = zeros(size(x1));  % z-values as zeros for 2D plotting in 3D space
% 
% % Plot the grid nodes in 3D
% figure;
% scatter3(x1, x2, z, 50, 'filled', 'MarkerFaceColor', 'b');  % Nodes in blue
% xlabel('x');
% ylabel('y');
% zlabel('z');
% title('10x10 Grid of Nodes with Connections and Random Probabilities');
% grid on;
% hold on;
% 
% % Define neighbor offsets (excluding the node itself)
% neighbor_offsets = [
%     -1, -1;
%     -1,  0;
%     -1,  1;
%      0, -1;
%      0,  1;
%      1, -1;
%      1,  0;
%      1,  1;
% ];
% 
% % Loop over each node to connect it to its neighbors
% for idx = 1:length(x1)
%     current_x = x1(idx);
%     current_y = x2(idx);
%     current_z = z(idx);
% 
%     % Loop over neighbor offsets
%     for n = 1:size(neighbor_offsets, 1)
%         neighbor_x = current_x + neighbor_offsets(n, 1);
%         neighbor_y = current_y + neighbor_offsets(n, 2);
% 
%         % Check if neighbor is within grid bounds
%         if neighbor_x >= 0 && neighbor_x <= 9 && neighbor_y >= 0 && neighbor_y <= 9
%             % Ensure we only connect to neighbors in one direction to avoid duplicates
%             if (neighbor_x > current_x) || (neighbor_x == current_x && neighbor_y > current_y)
%                 % Find the index of the neighbor node
%                 neighbor_idx = find(x1 == neighbor_x & x2 == neighbor_y);
%                 if ~isempty(neighbor_idx)
%                     % Assign a random probability to the connection (between 0 and 1)
%                     prob_connection = rand();
% 
%                     % Plot a line between the current node and the neighbor
%                     x_coords = [current_x, neighbor_x];
%                     y_coords = [current_y, neighbor_y];
%                     z_coords = [current_z, current_z];  % z = 0 for both nodes
% 
%                     % Adjust line width based on probability (optional)
%                     line_width = 1 + 2 * prob_connection;  % Line width between 1 and 3
% 
%                     % Plot the line
%                     plot3(x_coords, y_coords, z_coords, 'r-', 'LineWidth', line_width);  % Red lines
% 
%                     % Display the probability near the connection in white color
%                     mid_x = mean(x_coords);
%                     mid_y = mean(y_coords);
%                     mid_z = mean(z_coords);
%                     text(mid_x, mid_y, mid_z + 0.1, sprintf('%.2f', prob_connection), 'FontSize', 8, 'Color', 'w', 'HorizontalAlignment', 'center');
%                 end
%             end
%         end
%     end
% end
% 
% hold off;
% view(45, 30);  % Adjust the viewing angle
% 


% MATLAB Code: 10x10 Grid with Node and Edge Probabilities, and Average Probability Displayed at Each Node

% Clear workspace and figures
% clear;
% close all;
% 
% % Set random seed for reproducibility (optional)
% rng(1);
% 
% % Define the grid size
% N = 10;
% 
% % Define the grid coordinates
% [x1_grid, x2_grid] = meshgrid(1:N, 1:N);
% 
% % Flatten the grids for plotting
% x1 = x1_grid(:);  % x-coordinates of nodes
% x2 = x2_grid(:);  % y-coordinates of nodes
% z = zeros(size(x1));  % z-values as zeros for 2D plotting in 3D space
% 
% % Initialize node probabilities (random probabilities between 0 and 1)
% node_probs = rand(N, N);
% 
% % Initialize edge probabilities (N x N x 8 array)
% edge_probs = zeros(N, N, 8);
% 
% % Define neighbor offsets (8 directions)
% neighbor_offsets = [
%     -1, -1;  % Up-Left
%     -1,  0;  % Up
%     -1,  1;  % Up-Right
%      0, -1;  % Left
%      0,  1;  % Right
%      1, -1;  % Down-Left
%      1,  0;  % Down
%      1,  1;  % Down-Right
% ];
% 
% % Assign edge probabilities
% for i = 1:N
%     for j = 1:N
%         for k = 1:8
%             di = neighbor_offsets(k, 1);
%             dj = neighbor_offsets(k, 2);
%             ni = i + di;
%             nj = j + dj;
%             if ni >= 1 && ni <= N && nj >= 1 && nj <= N
%                 edge_probs(i, j, k) = rand();
%             else
%                 edge_probs(i, j, k) = 0;  % No neighbor; probability is zero
%             end
%         end
%     end
% end
% 
% % Compute average probabilities for each node
% avg_probs = zeros(N, N);
% for i = 1:N
%     for j = 1:N
%         total_prob = node_probs(i,j) + sum(edge_probs(i,j,:));
%         avg_probs(i,j) = total_prob / 9;  % Divide by 9 even for edge nodes
%     end
% end
% 
% % Plot the grid nodes in 3D
% figure;
% scatter3(x1, x2, z, 50, 'filled', 'MarkerFaceColor', 'b');  % Nodes in blue
% xlabel('x');
% ylabel('y');
% zlabel('z');
% title('10x10 Grid with Node and Edge Probabilities');
% grid on;
% hold on;
% 
% % Plot the connections with edge probabilities
% for i = 1:N
%     for j = 1:N
%         current_x = x1_grid(i,j);
%         current_y = x2_grid(i,j);
%         current_z = 0;
%         for k = 1:8
%             di = neighbor_offsets(k,1);
%             dj = neighbor_offsets(k,2);
%             ni = i + di;
%             nj = j + dj;
%             if ni >= 1 && ni <= N && nj >= 1 && nj <= N
%                 % To avoid duplicate lines, only plot if (ni > i) or (ni == i and nj > j)
%                 if (ni > i) || (ni == i && nj > j)
%                     neighbor_x = x1_grid(ni,nj);
%                     neighbor_y = x2_grid(ni,nj);
%                     neighbor_z = 0;
% 
%                     % Edge probability
%                     prob_connection = edge_probs(i,j,k);
% 
%                     % Plot line between current node and neighbor
%                     x_coords = [current_x, neighbor_x];
%                     y_coords = [current_y, neighbor_y];
%                     z_coords = [current_z, neighbor_z];  % z = 0 for both nodes
% 
%                     % Adjust line width based on probability (optional)
%                     line_width = 1 + 2 * prob_connection;  % Line width between 1 and 3
% 
%                     % Plot the line
%                     plot3(x_coords, y_coords, z_coords, 'r-', 'LineWidth', line_width);  % Red lines
% 
%                     % Display the probability near the connection in white color
%                     mid_x = mean(x_coords);
%                     mid_y = mean(y_coords);
%                     mid_z = mean(z_coords);
%                     text(mid_x, mid_y, mid_z + 0.1, sprintf('%.2f', prob_connection), 'FontSize', 8, 'Color', 'w', 'HorizontalAlignment', 'center');
%                 end
%             end
%         end
%     end
% end
% 
% % Display the average probability at each node in yellow color
% for i = 1:N
%     for j = 1:N
%         node_x = x1_grid(i,j);
%         node_y = x2_grid(i,j);
%         node_z = 0;
%         avg_prob = avg_probs(i,j);
%         text(node_x, node_y, node_z + 0.2, sprintf('%.2f', avg_prob), 'FontSize', 8, 'Color', 'y', 'HorizontalAlignment', 'center');
%     end
% end
% 
% hold off;
% view(45, 30);  % Adjust the viewing angle



% MATLAB Code: 10x10 Grid with Connections, Random Probabilities, and Action Simulation

% Clear workspace and figures
clear;
close all;
clc;

% Set random seed for reproducibility (optional)
rng('shuffle');  % Use 'shuffle' for different results each run

% --------------------- 1. Create the 10x10 Grid ---------------------

% Define grid size
N = 10;

% Define the grid coordinates
[x1_grid, x2_grid] = meshgrid(1:N, 1:N);

% Flatten the grids for plotting
x1 = x1_grid(:);  % x-coordinates of nodes
x2 = x2_grid(:);  % y-coordinates of nodes
z = zeros(size(x1));  % z-values as zeros for 2D plotting in 3D space

% Initialize node probabilities (random probabilities between 0 and 1)
node_probs = rand(N, N);

% Initialize edge probabilities (N x N x 8 array)
edge_probs = zeros(N, N, 8);

% Define neighbor offsets (8 directions)
neighbor_offsets = [
    -1, -1;  % Up-Left
    -1,  0;  % Up
    -1,  1;  % Up-Right
     0, -1;  % Left
     0,  1;  % Right
     1, -1;  % Down-Left
     1,  0;  % Down
     1,  1;  % Down-Right
];

% Assign edge probabilities
for i = 1:N
    for j = 1:N
        for k = 1:8
            di = neighbor_offsets(k, 1);
            dj = neighbor_offsets(k, 2);
            ni = i + di;
            nj = j + dj;
            if ni >= 1 && ni <= N && nj >= 1 && nj <= N
                edge_probs(i, j, k) = rand();
            else
                edge_probs(i, j, k) = 0;  % No neighbor; probability is zero
            end
        end
    end
end

% --------------------- 2. Compute Average Probabilities ---------------------

% Compute average probabilities for each node
avg_probs = zeros(N, N);
for i = 1:N
    for j = 1:N
        total_prob = node_probs(i,j) + sum(edge_probs(i,j,:));
        avg_probs(i,j) = total_prob / 9;  % Divide by 9 even for edge nodes
    end
end

% --------------------- 3. Plot the Grid Nodes ---------------------

% Create a new figure for the grid
figure;
scatter3(x1, x2, z, 50, 'filled', 'MarkerFaceColor', 'b');  % Nodes in blue
xlabel('x');
ylabel('y');
zlabel('z');
title('10x10 Grid of Nodes with Connections and Probabilities');
grid on;
hold on;

% --------------------- 4. Plot Connections with Edge Probabilities ---------------------

% Loop over each node to connect it to its neighbors
for i = 1:N
    for j = 1:N
        current_x = x1_grid(i,j);
        current_y = x2_grid(i,j);
        current_z = 0;
        for k = 1:8
            di = neighbor_offsets(k,1);
            dj = neighbor_offsets(k,2);
            ni = i + di;
            nj = j + dj;
            if ni >= 1 && ni <= N && nj >= 1 && nj <= N
                % To avoid duplicate lines, only connect if neighbor is in a higher index direction
                if (ni > i) || (ni == i && nj > j)
                    neighbor_x = x1_grid(ni,nj);
                    neighbor_y = x2_grid(ni,nj);
                    neighbor_z = 0;
                    
                    % Edge probability
                    prob_connection = edge_probs(i,j,k);
                    
                    % Plot the line with line width based on probability
                    x_coords = [current_x, neighbor_x];
                    y_coords = [current_y, neighbor_y];
                    z_coords = [current_z, neighbor_z];
                    line_width = 1 + 2 * prob_connection;  % Line width between 1 and 3
                    
                    plot3(x_coords, y_coords, z_coords, 'r-', 'LineWidth', line_width);  % Red lines
                    
                    % Display the probability near the connection in white color
                    mid_x = mean(x_coords);
                    mid_y = mean(y_coords);
                    mid_z = mean(z_coords);
                    text(mid_x, mid_y, mid_z + 0.1, sprintf('%.2f', prob_connection), ...
                        'FontSize', 8, 'Color', 'w', 'HorizontalAlignment', 'center');
                end
            end
        end
    end
end

% --------------------- 5. Display Average Probabilities at Nodes ---------------------

% Display the average probability at each node in yellow color
for i = 1:N
    for j = 1:N
        node_x = x1_grid(i,j);
        node_y = x2_grid(i,j);
        node_z = 0;
        avg_prob = avg_probs(i,j);
        text(node_x, node_y, node_z + 0.2, sprintf('%.2f', avg_prob), ...
            'FontSize', 8, 'Color', 'y', 'HorizontalAlignment', 'center');
    end
end

% --------------------- 6. Select a Random Initial Node ---------------------

% Select a random node as the initial point
initial_idx = randi(length(x1));  % Random index between 1 and N*N
initial_x = x1(initial_idx);
initial_y = x2(initial_idx);
initial_z = z(initial_idx);

% Highlight the initial node with a distinct marker (e.g., green star)
scatter3(initial_x, initial_y, initial_z, 100, 'filled', 'MarkerFaceColor', 'g');
text(initial_x, initial_y, initial_z + 0.3, 'Start', 'FontSize', 10, 'Color', 'k', ...
    'HorizontalAlignment', 'center');

% --------------------- 7. Simulate Three Actions (Steps) ---------------------

% Define possible actions (including staying in place)
actions = [
    -1, -1;  % Up-Left
    -1,  0;  % Up
    -1,  1;  % Up-Right
     0, -1;  % Left
     0,  0;  % Stay
     0,  1;  % Right
     1, -1;  % Down-Left
     1,  0;  % Down
     1,  1;  % Down-Right
];

% Initialize path
path_x = initial_x;
path_y = initial_y;

current_x = initial_x;
current_y = initial_y;

% Simulate three steps
num_steps = 3;
for step = 1:num_steps
    % Choose a random action (1 to 9)
    action_idx = randi(size(actions,1));
    action = actions(action_idx,:);
    
    % Determine new position
    new_x = current_x + action(1);
    new_y = current_y + action(2);
    
    % Check if new position is within grid bounds
    if new_x >= 1 && new_x <= N && new_y >=1 && new_y <= N
        current_x = new_x;
        current_y = new_y;
    else
        % If action leads out of bounds, stay in place
        % Alternatively, you could choose another action
        disp(['Step ', num2str(step), ': Action leads out of bounds. Staying at (', ...
            num2str(current_x), ', ', num2str(current_y), ').']);
    end
    
    % Append to path
    path_x = [path_x, current_x];
    path_y = [path_y, current_y];
    
    % Highlight the new node in a different color (e.g., magenta)
    scatter3(current_x, current_y, 0, 100, 'filled', 'MarkerFaceColor', 'm');
    text(current_x, current_y, 0.3, sprintf('Step %d', step), 'FontSize', 8, ...
        'Color', 'k', 'HorizontalAlignment', 'center');
end

% --------------------- 8. Plot the Path ---------------------

% Plot the path as a connected line (e.g., black dashed line)
plot3(path_x, path_y, zeros(size(path_x)), 'k--', 'LineWidth', 2);

% Mark the final node with a distinct marker (e.g., red diamond)
final_x = path_x(end);
final_y = path_y(end);
final_z = 0;
scatter3(final_x, final_y, final_z, 100, 'filled', 'MarkerFaceColor', 'r');
text(final_x, final_y, final_z + 0.3, 'End', 'FontSize', 10, 'Color', 'k', ...
    'HorizontalAlignment', 'center');

% --------------------- 9. Final Adjustments ---------------------

% Adjust colormap
colormap('jet');
colorbar;

% Adjust axis limits
xlim([0.5, N+0.5]);
ylim([0.5, N+0.5]);
zlim([-0.5, 1]);  % Adjust z-axis to accommodate text labels

% Adjust the viewing angle for better visualization
view(45, 30);  % Azimuth and elevation

hold off;
