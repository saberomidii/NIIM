import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as patches



# Defining problem
# Solving l function for states, input, and disturbance
# l(trajectory)
# trajectory(states, input, disturbance)

# Dynamic function 
def dubins_car(state, input_omega, adversial_disturbance, v, dt):
    """
    Simple second-order dynamic function for Dubin's car.

    Parameters:
    - state (array-like): Current state of the car [x, y, theta].
    - input_omega (float): Steering rate (omega).
    - adversial_disturbance (float): Adversarial disturbance added to the steering rate.
    - v (float): Constant speed of the car.
    - dt (float): Time step.

    Returns:
    - next_state (numpy.ndarray): Updated state [x_next, y_next, theta_next].
    """
    # Ensure the state is a NumPy array for element-wise operations
    state = np.array(state)
    
    # Compute the next state components
    x_next = state[0] + v * np.cos(state[2]) * dt
    y_next = state[1] + v * np.sin(state[2]) * dt
    theta_next = state[2] + (input_omega + adversial_disturbance) * dt

    # Create the next state array
    next_state = np.array([x_next, y_next, theta_next])

    return next_state


import numpy as np

def signed_distance(position, min_vals, max_vals):
    """
    Computes the signed distance for a single point relative to a rectangular set.

    Parameters:
    ----------
    position : array-like
        A 1D array or list with two elements representing the point's coordinates [x, y].
    
    min_vals : array-like
        A 1D array or list with two elements representing the minimum bounds of the rectangle [x_min, y_min].
    
    max_vals : array-like
        A 1D array or list with two elements representing the maximum bounds of the rectangle [x_max, y_max].

    Returns:
    -------
    float
        The signed distance value `s`.
        - Negative value indicates the point is inside the set.
        - Positive value indicates the point is outside the set.

    Examples:
    --------
    >>> position = [0, 0]
    >>> min_vals = [-5, -3]
    >>> max_vals = [3, 5]
    >>> s = signed_distance(position, min_vals, max_vals)
    >>> print(s)
    0.0

    >>> position = [4, 0]
    >>> s = signed_distance(position, min_vals, max_vals)
    >>> print(s)
    1.0

    >>> position = [0, 4]
    >>> s = signed_distance(position, min_vals, max_vals)
    >>> print(s)
    1.0

    >>> position = [0, 0]
    >>> s = signed_distance(position, min_vals, max_vals)
    >>> print(s)
    0.0

    >>> position = [-6, -4]
    >>> s = signed_distance(position, min_vals, max_vals)
    >>> print(s)
    2.8284271247461903
    """

    # Convert inputs to NumPy arrays for vectorized operations
    position = np.array(position)
    min_vals = np.array(min_vals)
    max_vals = np.array(max_vals)

    # Extract x and y coordinates
    x, y = position

    # Check if the point is inside the rectangular set
    inside_x = min_vals[0] <= x <= max_vals[0]
    inside_y = min_vals[1] <= y <= max_vals[1]

    if inside_x and inside_y:
        # Inside the set

        # Compute distances to the boundaries
        distance_to_min_x = x - min_vals[0]
        distance_to_max_x = max_vals[0] - x
        distance_to_min_y = y - min_vals[1]
        distance_to_max_y = max_vals[1] - y

        # Find the minimum distance to any boundary
        min_distance_to_boundary = min(distance_to_min_x, distance_to_max_x,
                                       distance_to_min_y, distance_to_max_y)

        # Signed distance is negative of the minimum distance to boundary
        s = -min_distance_to_boundary
    else:
        # Outside the set

        # Initialize distance components
        dx = 0
        dy = 0

        # Compute distance in x-direction
        if x < min_vals[0]:
            dx = min_vals[0] - x
        elif x > max_vals[0]:
            dx = x - max_vals[0]

        # Compute distance in y-direction
        if y < min_vals[1]:
            dy = min_vals[1] - y
        elif y > max_vals[1]:
            dy = y - max_vals[1]

        # Compute the Euclidean distance to the set
        s = np.sqrt(dx**2 + dy**2)
    return s


# Time parameters 
simulation_time=1
dt = 0.1  # Time step (set to 0.01 as per your requirement)
time_step=simulation_time/dt
time_index=0

# Define minimum and maximum values as vectors
min_vals_state = np.array([-1, -2, -np.pi/6])
max_vals_state = np.array([1, 2, np.pi/6])
min_input = -1
max_input = 1

# Create input and state lists with a step of 0.01
input_list = np.arange(min_input, max_input + dt, dt)  # Inputs from -1 to 1
state_1_list = np.arange(min_vals_state[0], max_vals_state[0] + dt, dt)  # State1 from -5 to 3
state_2_list = np.arange(min_vals_state[1], max_vals_state[1] + dt, dt)  # State2 from -3 to 3
state_3_list = np.arange(min_vals_state[2], max_vals_state[2] + dt, dt)  # State3 from -pi/3 to pi/3



#Dynamic Parameters 
v = 1;                    # Constant speed

# random parameters
mu=0
sigma=10
population=10000



# Initialize dataset list to store each trajectory as a unique case
dataset = []
trajectory_id = 0  # Start the ID for each closed-loop trajectory

# Loop through all possible combinations of u, x1, x2, x3
for index_state_3 in state_3_list:
    for index_state_2 in state_2_list:
        for index_state_1 in state_1_list:
            for index_input in input_list:
                
                # Set up initial values for each closed-loop simulation
                input_value = index_input
                state = [np.array([index_state_1, index_state_2, index_state_3])]
                L = []  # Collect L values over time
                V = []  # Collect V values over time
                disturbance_history = []  # Track disturbance values over time
                state_trajectory = [state[0]]  # Store the full state trajectory

                # Initial signed distance calculation
                l = -signed_distance([index_state_1, index_state_2], min_vals_state, max_vals_state)
                L.append(l)

                time_index = 0  # Time index for closed loop
 

 
                # Closed-loop simulation
                while time_index <= time_step:

                    current_state = state[time_index]

                    # Generate a disturbance for this time step
                    adversarial_disturbance = np.random.normal(mu, sigma, 1)[0]
                    
                    #### solving optimization problem #######


                    # Compute next state using the Dubins car dynamics
                    next_state = dubins_car(current_state, input_value, adversarial_disturbance, v, dt)

                    # Calculate new signed distance and update L
                    l = -signed_distance([next_state[0], next_state[1]], min_vals_state, max_vals_state)
                    L.append(l)

                    # Calculate V as the minimum L so far
                    v = min(L)
                    V.append(v)

                    # Append next state and disturbance to respective lists
                    state.append(next_state)
                    state_trajectory.append(next_state)  # Track the full trajectory
                    disturbance_history.append(adversarial_disturbance)

                    time_index += 1

                # Append the results for this unique combination to the dataset
                dataset.append({
                    'trajectory_id': trajectory_id,
                    'u': input_value,
                    'initial_state': [index_state_1, index_state_2, index_state_3],
                    'state_trajectory': state_trajectory,
                    'L_list': L,
                    'V_list': V,
                    'disturbance_list': disturbance_history
                })
                trajectory_id += 1  # Increment trajectory ID for the next unique combination

# Convert the dataset to a DataFrame
df = pd.DataFrame(dataset)

# Display or save the DataFrame
# Extract V_list as a separate DataFrame
V_lists = df[['V_list']].copy()

print(V_lists)


last_elements = V_lists['V_list'].apply(lambda v: v[-1])
print(last_elements)


# Get the maximum value and its index
max_last_element_index = last_elements.idxmax()
max_last_element = last_elements[max_last_element_index]

# Display the results
print("Maximum last element:", max_last_element)
print("Index of maximum last element:", max_last_element_index)

# Print the row from the original dataset `df` using the found index
print(df.iloc[max_last_element_index])

# Extract the V_list for the row with the maximum last element
V_list_to_plot = df.iloc[max_last_element_index]['V_list']

# Extract the V_list and trajectory for the row with the maximum last element
V_list_to_plot = df.iloc[max_last_element_index]['V_list']
state_trajectory = np.array(df.iloc[max_last_element_index]['state_trajectory'])  # Convert to array for easy indexing

# Create a 1x2 figure with subplots
fig, axs = plt.subplots(1, 2, figsize=(16, 6))

# --- First Subplot: V List over Time ---
axs[0].plot(range(len(V_list_to_plot)), V_list_to_plot, linestyle='-', color='purple', marker='o')
axs[0].set_xlabel('Time Step')
axs[0].set_ylabel('V Value')
axs[0].set_title('V List over Time for Row with Maximum Last Element')
axs[0].grid(True)

# --- Second Subplot: Trajectory ---
x = state_trajectory[:, 0]
y = state_trajectory[:, 1]

axs[1].plot(x, y, linestyle='-', color='g', label='Trajectory')
axs[1].scatter(x[0], y[0], color='blue', s=100, label='Start')
axs[1].scatter(x[-1], y[-1], color='red', s=100, label='End')

# Define rectangle boundaries using min and max values
min_x, min_y = min_vals_state[:2]
max_x, max_y = max_vals_state[:2]
width = max_x - min_x
height = max_y - min_y
rect = patches.Rectangle((min_x, min_y), width, height, linewidth=2, edgecolor='red', facecolor='none', label='Boundary')
axs[1].add_patch(rect)

axs[1].set_xlabel('X')
axs[1].set_ylabel('Y')
axs[1].set_title('Trajectory with Boundary')
axs[1].grid(True)
axs[1].axis('equal')
axs[1].legend()

# Adjust layout for better spacing
plt.tight_layout()
plt.show()