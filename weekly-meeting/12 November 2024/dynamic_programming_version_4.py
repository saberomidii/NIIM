import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math

# ----------------------------
# 1. Define Dynamics and Cost Functions
# ----------------------------

def dynamics(state, action,d, delta_t):
    """
    Computes the next state for a simple two-state system.
    
    Parameters:
    - state (tuple): Current state as (x, v).
    - action (float): Acceleration input.
    - delta_t (float): Time increment.
    
    Returns:
    - tuple: Next state as (x_next, v_next).
    """
    x, v = state
    x_next = x + v * delta_t
    v_next = v + (d+action) * delta_t
    return (x_next, v_next)

def g(state):
    """
    Defines the cost function or signed distance function.
    
    Parameters:
    - state (tuple): Current state as (x, v).
    
    Returns:
    - float: Cost associated with the state.
    """
    x, v = state
    min_x, max_x = -1, 1
    min_v, max_v = -0.2, 0.2
    if min_x <= x <= max_x and min_v <= v <= max_v:
        # Inside the boundary: positive distance to the closest boundary
        return -min(x - min_x, max_x - x, v - min_v, max_v - v)
    else:
        # Outside the boundary: Euclidean distance to the boundary
        dx = max(min_x - x, 0, x - max_x)
        dv = max(min_v - v, 0, v - max_v)
        return math.sqrt(dx**2 + dv**2)

def generate_bounded_normal_samples(mean, variance, lower_bound, upper_bound, num_samples):
    """
    Generate samples from a normal distribution with specified mean and variance,
    constrained by upper and lower bounds.
    
    Parameters:
    - mean (float): The mean of the normal distribution.
    - variance (float): The variance of the normal distribution.
    - lower_bound (float): The lower bound for the samples.
    - upper_bound (float): The upper bound for the samples.
    - num_samples (int): The number of samples to generate.
    
    Returns:
    - np.ndarray: An array of samples within the bounds.
    """
    std_dev = np.sqrt(variance)
    samples = []

    while len(samples) < num_samples:
        sample = np.random.normal(mean, std_dev)
        if lower_bound <= sample <= upper_bound:
            samples.append(sample)

    return np.array(samples)


# Define disturbance function
def disturbance_function(step, mean=0, variance=0.01, lower_bound=-0.05, upper_bound=0.05):
    """
    Generates a disturbance value for a given time step using a bounded normal distribution.

    Parameters:
    - step (int): Current time step (unused, included for consistency).
    - mean (float): Mean of the normal distribution.
    - variance (float): Variance of the normal distribution.
    - lower_bound (float): Lower bound of the disturbance values.
    - upper_bound (float): Upper bound of the disturbance values.

    Returns:
    - float: A single disturbance value within the specified bounds.
    """
    samples = generate_bounded_normal_samples(mean, variance, lower_bound, upper_bound, num_samples=1)
    return samples[0]  # Return the single sample as a disturbance value



# Example usage
mean = 0
variance = 1
lower_bound = -0.5
upper_bound = 0.5
num_samples = 1




# ----------------------------
# 2. Define Grids and Actions
# ----------------------------

# Define the grid parameters
lower_value = [-1.1, -0.25, -1]  # [x_min, v_min, action_min]
upper_value = [1.1, 0.25, 1]      # [x_max, v_max, action_max]
num_points = [50, 50, 50]       # [x_points, v_points, action_points]

# Generate grids using linspace
x_grid = np.linspace(lower_value[0], upper_value[0], num=num_points[0])
v_grid = np.linspace(lower_value[1], upper_value[1], num=num_points[1])
actions = np.linspace(lower_value[2], upper_value[2], num=num_points[2])

# Define time parameters
time_steps = 10
delta_t = 0.01

# ----------------------------
# 3. Initialize Value Function and Policy
# ----------------------------

# Initialize value function as a 3D NumPy array filled with infinity
# Shape: (time_steps + 1, num_v, num_x)
V = np.full((time_steps + 1, len(v_grid), len(x_grid)), np.inf)

# Initialize policy as a 3D NumPy array filled with NaN
# Shape: (time_steps, num_v, num_x)
policy = np.full((time_steps, len(v_grid), len(x_grid)), np.nan)

# Terminal condition: At the final time step, the value is 0


# ----------------------------
# 4. Backward Iteration for Value Function and Policy
# ----------------------------

def get_indices(x, v, x_grid, v_grid):
    """
    Given state (x, v), find the nearest indices in x_grid and v_grid.
    """
    x_idx = np.abs(x_grid - x).argmin()
    v_idx = np.abs(v_grid - v).argmin()
    return v_idx, x_idx  # rows, columns

V[time_steps, :, :] = None

# Loop to compute the optimal policy
for k in range(time_steps - 1, -1, -1):
    print(f"Calculating optimal policy for time step {k}")
    for i, v in enumerate(v_grid):
        for j, x in enumerate(x_grid):
            state = (x, v)
            
            max_min_value = np.inf
            best_action = None

            for u in actions:
                d = generate_bounded_normal_samples(mean, variance, lower_bound, upper_bound, num_samples)
                next_state = dynamics(state, u,d, delta_t)
                x_next, v_next = next_state
                # Check if next_state is within the grid boundaries
                if (lower_value[0] <= x_next <= upper_value[0]) and (lower_value[1] <= v_next <= upper_value[1]):
                    # Find nearest indices
                    v_next_idx, x_next_idx = get_indices(x_next, v_next, x_grid, v_grid)
                    current_value = V[k + 1, v_next_idx, x_next_idx]
                else:
                    current_value = np.inf  # Assign a high penalty for out of bounds

                # Compute future value
                future_value = min(g(state), current_value)
                # print(current_value)
                if future_value < max_min_value:
                    max_min_value = future_value
                    best_action = u

            # Update the value function and policy
            V[k, i, j] = max_min_value
            policy[k, i, j] = best_action

# ----------------------------
# 5. Prepare Data for Surface Plot
# ----------------------------

# Access the value function at time=0
current_V = V[0, :, :]
current_policy = policy[0, :, :]

# Create meshgrid for X and V
X_mesh, Y_mesh = np.meshgrid(x_grid, v_grid)  # Y_mesh corresponds to v_grid

# Z for surface plot
Z = current_V

# U for optimal policy (actions)
U = current_policy

# ----------------------------
# 6. Generate Comprehensive 3D Plots
# ----------------------------

# Initialize the figure with multiple subplots
fig = plt.figure(figsize=(20, 15))

# ----------------------------
# Subplot 1: 3D Surface Plot
# ----------------------------
ax1 = fig.add_subplot(2, 2, 1, projection='3d')
surf1 = ax1.plot_surface(X_mesh, Y_mesh, Z, cmap='viridis', edgecolor='none')
ax1.set_xlabel('State 1 (x)')
ax1.set_ylabel('State 2 (v)')
ax1.set_zlabel('Value V')
ax1.set_title('3D Surface Plot of V at Time=0')
ax1.view_init(elev=30, azim=225)  # Adjust viewing angle

# ----------------------------
# Subplot 2: V vs. X Plot
# ----------------------------
ax2 = fig.add_subplot(2, 2, 2)
# Select a fixed v, e.g., median v
fixed_v_idx = len(v_grid) // 2
fixed_v = v_grid[fixed_v_idx]
V_vs_X = current_V[fixed_v_idx, :]
ax2.plot(x_grid, V_vs_X, color='blue')
ax2.set_xlabel('State 1 (x)')
ax2.set_ylabel('Value V')
ax2.set_title(f'V vs. X at v = {fixed_v:.2f}')
ax2.grid(True)

# ----------------------------
# Subplot 3: V vs. v Plot
# ----------------------------
ax3 = fig.add_subplot(2, 2, 3)
# Select a fixed x, e.g., median x
fixed_x_idx = len(x_grid) // 2
fixed_x = x_grid[fixed_x_idx]
V_vs_v = current_V[:, fixed_x_idx]
ax3.plot(v_grid, V_vs_v, color='green')
ax3.set_xlabel('State 2 (v)')
ax3.set_ylabel('Value V')
ax3.set_title(f'V vs. v at x = {fixed_x:.2f}')
ax3.grid(True)

# ----------------------------
# Subplot 4: Top View with Contour and Policy Arrows
# ----------------------------
ax4 = fig.add_subplot(2, 2, 4)
# Create a contour plot
contour = ax4.contourf(X_mesh, Y_mesh, Z, cmap='viridis', levels=50)
ax4.set_xlabel('State 1 (x)')
ax4.set_ylabel('State 2 (v)')
ax4.set_title('Top View: X vs. V with V as Contour')

# Add a color bar for contour
cbar = fig.colorbar(contour, ax=ax4, shrink=0.6, aspect=20)
cbar.set_label('Value V')


plt.tight_layout()
plt.show()

# ----------------------------
# 7. Simulation Using the Optimal Policy (Updated to Track Value Function)
# ----------------------------
# Define initial state
initial_state = (-0.85, -0.1)  # Example: x = -0.5, v = 0.0
# Define boundary rectangle
boundary_x = [-1, 1, 1, -1, -1]  # Rectangle corners for x
boundary_v = [-0.2, -0.2, 0.2, 0.2, -0.2]  # Rectangle corners for v
# Updated simulation function with disturbance tracking
def run_simulation_with_value_tracking_and_disturbance(
    initial_state, policy, x_grid, v_grid, V, dynamics, delta_t, time_steps, disturbance_function
):
    """
    Runs a simulation using the optimal policy, tracks inputs (actions), disturbance, and logs the value function over time.
    
    Parameters:
    - initial_state (tuple): Starting state as (x0, v0).
    - policy (numpy.ndarray): Optimal policy array of shape (time_steps, num_v, num_x).
    - x_grid (numpy.ndarray): Grid for state variable x.
    - v_grid (numpy.ndarray): Grid for state variable v.
    - V (numpy.ndarray): Value function array.
    - dynamics (function): Function to compute next state.
    - delta_t (float): Time increment.
    - time_steps (int): Number of time steps.
    - disturbance_function (function): Function to generate disturbance values over time.
    
    Returns:
    - trajectory (list): List of states (x, v) over time.
    - inputs (list): List of actions (inputs) over time.
    - value_over_time (list): Value of the state at each time step.
    - disturbances (list): List of disturbance values over time.
    """
    trajectory = [initial_state]
    inputs = []  # To track the actions taken at each step
    value_over_time = []  # To track the value function over time
    disturbances = []  # To track disturbances over time
    current_state = initial_state

    for k in range(time_steps):
        x, v = current_state
        # Find nearest grid indices
        x_idx = np.abs(x_grid - x).argmin()
        v_idx = np.abs(v_grid - v).argmin()
        # Get the value of the current state from the value function
        value_over_time.append(V[k, v_idx, x_idx])
        # Get the optimal action
        u = policy[k, v_idx, x_idx]
        if np.isnan(u):
            print(f"No optimal action found for state ({x:.2f}, {v:.2f}) at time step {k}. Stopping simulation.")
            break
        # Generate disturbance for the current time step
        d = disturbance_function(1, mean=0, variance=5, lower_bound=-1, upper_bound=1)
        disturbances.append(d)
        # Apply dynamics to get next state
        next_state = dynamics(current_state, u, d, delta_t)
        trajectory.append(next_state)
        inputs.append(u)  # Store the action
        current_state = next_state

    return trajectory, inputs, value_over_time, disturbances




# Run simulation with disturbances
trajectory, inputs, value_over_time, disturbances = run_simulation_with_value_tracking_and_disturbance(
    initial_state, policy, x_grid, v_grid, V, dynamics, delta_t, time_steps, disturbance_function
)

# Extract x and v from trajectory
trajectory_x = [state[0] for state in trajectory]
trajectory_v = [state[1] for state in trajectory]
time_points = np.arange(len(inputs)) * delta_t  # Time points for inputs

# Plotting the results
fig, ax = plt.subplots(4, 1, figsize=(10, 24), gridspec_kw={'height_ratios': [3, 1, 1, 1]})

# ----------------------------
# Subplot 1: Trajectory on Top View with Contour
# ----------------------------
# Create a contour plot
contour = ax[0].contourf(X_mesh, Y_mesh, Z, cmap='viridis', levels=50)
ax[0].set_xlabel('State 1 (x)')
ax[0].set_ylabel('State 2 (v)')
ax[0].set_title('Simulation Trajectory on Top View with Contour')

# Plot the trajectory
ax[0].plot(trajectory_x, trajectory_v, color='red', marker='o', label='Trajectory')

# Highlight the initial point with a different color and larger marker size
ax[0].scatter(trajectory_x[0], trajectory_v[0], color='yellow', s=100, edgecolor='black', label='Initial Point', zorder=5)

# Add boundary lines
ax[0].plot(boundary_x, boundary_v, 'r--', label='Boundary')

# Add a contour line where value function is zero
zero_value_contour = ax[0].contour(X_mesh, Y_mesh, Z, levels=[0], colors='blue', linewidths=2, linestyles='dashed')
ax[0].clabel(zero_value_contour, fmt='Value=0', inline=True, fontsize=10)

# Add a legend
ax[0].legend()

# Add a color bar
cbar = fig.colorbar(contour, ax=ax[0], shrink=0.6, aspect=20)
cbar.set_label('Value V')

# ----------------------------
# Subplot 2: Inputs Over Time
# ----------------------------
ax[1].plot(time_points, inputs, marker='o', linestyle='-', color='blue', label='Inputs (Actions)')
ax[1].set_xlabel('Time (s)')
ax[1].set_ylabel('Input (Action)')
ax[1].set_title('Inputs (Actions) Over Time')
ax[1].grid(True)
ax[1].legend()

# ----------------------------
# Subplot 3: Value Function Over Time
# ----------------------------
ax[2].plot(time_points, value_over_time, marker='o', linestyle='-', color='green', label='Value Function')
ax[2].set_xlabel('Time (s)')
ax[2].set_ylabel('Value V')
ax[2].set_title('Value Function Over Time')
ax[2].grid(True)
ax[2].legend()

# ----------------------------
# Subplot 4: Disturbances Over Time
# ----------------------------
ax[3].plot(time_points, disturbances, marker='o', linestyle='-', color='purple', label='Disturbances')
ax[3].set_xlabel('Time (s)')
ax[3].set_ylabel('Disturbance')
ax[3].set_title('Disturbances Over Time')
ax[3].grid(True)
ax[3].legend()

# Adjust layout for better spacing
plt.tight_layout()

# Show plots
plt.show()