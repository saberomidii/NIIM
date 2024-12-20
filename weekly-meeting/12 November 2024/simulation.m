clc 
clear 
close all

%% defining problem 
% 1- solving l function for states, input, and disturbance 
% l(trajectory)
% trajectory(states,input, disturbance)

dt = 0.01;                % Time step (set to 0.01 as per your requirement)

% Define minimum and maximum values as vectors
min_vals_state = [-5; -3;-pi/3];
max_vals_state = [3; 3;-pi/3];
min_input =-1;
max_input =1;


% Create input and state lists with a step of 0.01
input_list = min_input:dt:max_input;                  % Inputs from -1 to 1 
state_1_list = min_vals_state(1):dt:max_vals_state(1);   % State1 from -5 to 3 
state_2_list = min_vals_state(2):dt:max_vals_state(2);    % State2 from -3 to 3 
state_3_list = min_vals_state(3):dt:max_vals_state(3);    % State3 from -pi/3 to pi/3 



for index_state_3=state_3_list
    for index_state_2=state_2_list
        for index_state_1=state_1_list
            for index_input=input_list
            display([index_input,index_state_1,index_state_2,index_state_3])
            end
        end
    end
end

