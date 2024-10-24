% Simple Mu-Synthesis Example using MUSYN in MATLAB

% Clear workspace and command window
clear; clc;

% Define an uncertain parameter (mass with 20% uncertainty)
m = ureal('m', 1, 'Percentage', 20);  % Nominal value 1 kg, ±20% uncertainty

% Fixed parameters
k = 1;    % Spring constant (N/m)
b = 0.1;  % Damping coefficient (N·s/m)

% Define Laplace variable
s = tf('s');

% Define the uncertain plant transfer function P(s) = 1 / (m*s^2 + b*s + k)
Plant = 1 / (m * s^2 + b * s + k);

% Define weighting functions for performance (Wp) and control effort (Wu)
% Define Wp and Wu explicitly to avoid makeweight issues
Wp = tf([1, 0.1], [1, 10]);   % Performance weighting function
Wu = tf([1, 100], [1, 0.1]);  % Control effort weighting function

% Define the inputs and outputs for the systems
% Plant Input: 'uP' (control input plus disturbance)
% Plant Output: 'y' (measured output)
Plant.InputName = 'uP';
Plant.OutputName = 'y';

% Weighting functions inputs and outputs
Wp.InputName = 'e';  % Error signal (reference 'r' minus output 'y')
Wp.OutputName = 'z1';

Wu.InputName = 'u';  % Control input from controller
Wu.OutputName = 'z2';

% Define summing junctions
Sum1 = sumblk('e = r - y');   % Error signal
Sum2 = sumblk('uP = u + d');  % Plant input combines control 'u' and disturbance 'd'

% Combine all systems into a single augmented plant using 'append'
% This avoids name conflicts and preserves the individual I/O names
SystemList = append(Plant, Wp, Wu);

% Define the inputs and outputs for the overall interconnected system
% Inputs: external signals to the system (reference 'r' and disturbance 'd')
% Outputs: signals we want to measure or control ('z1', 'z2', 'y')
InputNames = {'r', 'd'};
OutputNames = {'z1', 'z2', 'y'};

% Build the interconnected system using the 'connect' function
% Include the sum blocks to define the interconnections between signals
P = connect(SystemList, Sum1, Sum2, InputNames, OutputNames);

% Define the number of measurements and controls
nmeas = 1;  % Number of measurements (outputs to controller), which is 'y'
ncont = 1;  % Number of control inputs from controller, which is 'u'

% Perform Mu-Synthesis using MUSYN
% Options for MUSYN (optional)
opt = musynOptions('Display', 'full');

% Run MUSYN to design the controller
[K, CLperf, info] = musyn(P, nmeas, ncont, opt);

% Display final robust performance achieved
disp('Final robust performance achieved:');
disp(info.MussvBnds);

% Analyze the closed-loop system
% Plot singular values of the closed-loop system
figure;
sigma(CLperf);
grid on;
title('Singular Value Plot of Closed-Loop System');

% Display the designed controller K
disp('Designed Controller K:');
tf(K)
