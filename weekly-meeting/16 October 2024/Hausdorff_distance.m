% Clear workspace and figures
clear; close all; clc;

% Generate first set of points (a circle)
theta = linspace(0, 2*pi, 100);
r = 5; % Radius of the circle
x1 = r * cos(theta);
y1 = r * sin(theta);
set1 = [x1', y1'];

% Generate second set of points (an ellipse)
a = 7; % Semi-major axis
b = 3; % Semi-minor axis
x2 = a * cos(theta) + 2; % Shift ellipse center to (2,0)
y2 = b * sin(theta);
set2 = [x2', y2'];

% Compute the pairwise distances between points in set1 and set2
D = pdist2(set1, set2);

% Compute the Hausdorff distance
hausdorff_dist = max([max(min(D, [], 2)), max(min(D, [], 1))]);

% Find the points that contribute to the Hausdorff distance
[~, idx1] = max(min(D, [], 2)); % Index in set1
[~, idx2] = max(min(D, [], 1)); % Index in set2

% Find the closest points in the other set
[~, idx2_closest] = min(D(idx1,:)); % Closest index in set2 to set1(idx1,:)
[~, idx1_closest] = min(D(:, idx2)); % Closest index in set1 to set2(idx2,:)

% Plot the two sets
figure;
plot(set1(:,1), set1(:,2), 'bo', 'MarkerFaceColor', 'b'); hold on;
plot(set2(:,1), set2(:,2), 'ro', 'MarkerFaceColor', 'r');
legend('Set 1 (Circle)', 'Set 2 (Ellipse)');
title(sprintf('Hausdorff Distance between Set 1 and Set 2: %.4f', hausdorff_dist));
xlabel('X');
ylabel('Y');
axis equal;

% Highlight the points contributing to the Hausdorff distance
plot(set1(idx1,1), set1(idx1,2), 'ks', 'MarkerSize', 10, 'LineWidth', 2);
plot(set2(idx2,1), set2(idx2,2), 'ks', 'MarkerSize', 10, 'LineWidth', 2);

% Draw lines showing the distances
% Line from set1(idx1,:) to its closest point in set2
line([set1(idx1,1), set2(idx2_closest,1)], [set1(idx1,2), set2(idx2_closest,2)], 'Color', 'k', 'LineStyle', '--');

% Line from set2(idx2,:) to its closest point in set1
line([set2(idx2,1), set1(idx1_closest,1)], [set2(idx2,2), set1(idx1_closest,2)], 'Color', 'k', 'LineStyle', '--');

% Annotate the points
text(set1(idx1,1), set1(idx1,2), '  P1', 'FontSize', 12, 'Color', 'k');
text(set2(idx2,1), set2(idx2,2), '  P2', 'FontSize', 12, 'Color', 'k');

% Display the figure
grid on;
