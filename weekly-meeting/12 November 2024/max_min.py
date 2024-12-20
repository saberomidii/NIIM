import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # For 3D plotting
import numpy as np

# Define the signed distance function g(s)
def g(s):
    x, y = s[:2]  # Only consider the x and y components
    min_x, max_x = -4, 4
    min_y, max_y = -3, 3
    if min_x <= x <= max_x and min_y <= y <= max_y:
        # Inside the boundary: negative distance to the closest edge
        return -min(x - min_x, max_x - x, y - min_y, max_y - y)
    else:
        # Outside the boundary: positive distance to the closest edge
        dx = max(min_x - x, 0, x - max_x)
        dy = max(min_y - y, 0, y - max_y)
        return math.sqrt(dx**2 + dy**2)

# Dubin's car dynamics function
def f(s, u, delta_t):
    x, y, theta = s
    v = 1.0  # Constant speed
    x_next = x + v * math.cos(theta) * delta_t
    y_next = y + v * math.sin(theta) * delta_t
    theta_next = ((theta + u * delta_t + math.pi) % (2 * math.pi)) - math.pi  # Normalize theta to [-pi, pi]
    return [x_next, y_next, theta_next]

# Discretize actions
def discretize_actions(start, stop, step):
    return [round(u, 10) for u in np.arange(start, stop + step, step)]

# Initialize a grid of x, y, and theta states within a range
def initialize_grid(min_val, max_val, step):
    return [round(x, 10) for x in np.arange(min_val, max_val + step, step)]

# Compute the optimal policy using max-min optimization
def compute_optimal_policy(time_steps, delta_t, action_step, grid_step):
    # Define grids
    x_grid = initialize_grid(-4, 4, grid_step)
    y_grid = initialize_grid(-3, 3, grid_step)
    theta_grid = initialize_grid(-math.pi / 3, math.pi / 3, 0.5)  # Theta grid
    actions = discretize_actions(-1, 1, action_step)

    # Initialize value function and policy
    V = [{} for _ in range(time_steps + 1)]
    policy = {}

    # Terminal condition: At the final time step, value is based on g(state)
    for x in x_grid:
        for y in y_grid:
            for theta in theta_grid:
                state = (x, y, theta)
                V[time_steps][state] = 0

    # Backward iteration to compute optimal policy
    for k in range(time_steps - 1, -1, -1):
        for x in x_grid:
            for y in y_grid:
                for theta in theta_grid:
                    state = (x, y, theta)
                    max_min_value = float('-inf')
                    best_action = None

                    for u in actions:
                        next_state = f([x, y, theta], u, delta_t)
                        x_next = round(min(max(next_state[0], -4), 4), 10)
                        y_next = round(min(max(next_state[1], -3), 3), 10)
                        theta_next = round(((next_state[2] + math.pi) % (2 * math.pi)) - math.pi, 10)
                        next_state_rounded = (x_next, y_next, theta_next)

                        # Retrieve future value; default to inf if next state is invalid
                        future_value = V[k + 1].get(next_state_rounded, float('inf'))
                        current_value = min(g(state), future_value)

                        if current_value > max_min_value:
                            max_min_value = current_value
                            best_action = u

                    # Update value function and policy
                    if best_action is not None:
                        V[k][state] = max_min_value
                        policy[state] = best_action
                    else:
                        V[k][state] = g(state)
                        policy[state] = 0  # Default action if none is valid

    return V, policy

# Simulate trajectory following the computed policy
def simulate_trajectory(initial_state, policy, V, time_steps, delta_t, grid_step):
    x_grid = initialize_grid(-4, 4, grid_step)
    y_grid = initialize_grid(-3, 3, grid_step)
    theta_grid = initialize_grid(-math.pi / 3, math.pi / 3, 0.5)

    current_state = initial_state
    trajectory = [tuple(current_state)]
    value_over_time = []
    policy_over_time = []

    for k in range(time_steps):
        nearest_state = map_to_nearest_grid(current_state, x_grid, y_grid, theta_grid)
        optimal_action = policy.get(nearest_state, 0)
        next_state = f(current_state, optimal_action, delta_t)

        trajectory.append(tuple(next_state))
        value = V[k].get(nearest_state, float('inf'))
        value_over_time.append(value)
        policy_over_time.append(optimal_action)

        current_state = next_state

    return trajectory, value_over_time, policy_over_time

# Map state to nearest x, y, theta grid point
def map_to_nearest_grid(state, x_grid, y_grid, theta_grid):
    x, y, theta = state
    x_closest = min(x_grid, key=lambda grid_x: abs(grid_x - x))
    y_closest = min(y_grid, key=lambda grid_y: abs(grid_y - y))
    theta_closest = min(theta_grid, key=lambda grid_theta: abs(grid_theta - theta))
    return (x_closest, y_closest, theta_closest)

# Plot the value function in 3D
def plot_value_function(V, time_step):
    x_vals = []
    y_vals = []
    theta_vals = []
    value_vals = []

    V_k = V[time_step]
    for (x, y, theta), value in V_k.items():
        x_vals.append(x)
        y_vals.append(y)
        theta_vals.append(theta)
        value_vals.append(value)

    x_vals = np.array(x_vals)
    y_vals = np.array(y_vals)
    theta_vals = np.array(theta_vals)
    value_vals = np.array(value_vals)

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    sc = ax.scatter(x_vals, y_vals, theta_vals, c=value_vals, cmap='viridis')
    ax.set_xlabel('X Position')
    ax.set_ylabel('Y Position')
    ax.set_zlabel('Theta')
    ax.set_title(f'Value Function at Time Step {time_step}')
    fig.colorbar(sc, ax=ax, label='Value')
    plt.show()

# Parameters
time_steps = 10
delta_t = 0.1
action_step = 0.1
grid_step = 0.1
initial_state = [-1.5, 1.5, 0]

# Compute optimal policy and simulate trajectory
V, policy = compute_optimal_policy(time_steps, delta_t, action_step, grid_step)
trajectory, value_over_time, policy_over_time = simulate_trajectory(initial_state, policy, V, time_steps, delta_t, grid_step)

# Plot trajectory and other results
trajectory_x = [state[0] for state in trajectory]
trajectory_y = [state[1] for state in trajectory]


plt.figure(figsize=(16, 4))
plt.subplot(1, 3, 1)
plt.plot(trajectory_x, trajectory_y, marker='o', label="Trajectory")
# Add rectangular boundary
plt.plot([-4, 4, 4, -4, -4], [-3, -3, 3, 3, -3], 'r--', label="Boundary")
plt.title("Trajectory with Boundary")
plt.xlabel("X Position")
plt.ylabel("Y Position")
plt.grid(True)
plt.legend()

plt.subplot(1, 3, 2)
plt.plot(range(len(value_over_time)), value_over_time, marker='o')
plt.title("Value Over Time")
plt.xlabel("Time Step")
plt.ylabel("Value")
plt.grid(True)

plt.subplot(1, 3, 3)
plt.plot(range(len(policy_over_time)), policy_over_time, marker='o', color='orange')
plt.title("Policy Over Time")
plt.xlabel("Time Step")
plt.ylabel("Action")
plt.grid(True)

plt.tight_layout()
plt.show()
# Plot value function in 3D
plot_value_function(V, 0)  # Plot at time step 0
