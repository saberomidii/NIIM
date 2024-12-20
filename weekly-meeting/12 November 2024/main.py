import math
import matplotlib.pyplot as plt

# Define the cost function or signed distance function
def g(s):
    x, y = s[:2]  # Only consider the x and y components
    min_x, max_x = -4, 4
    min_y, max_y = -3, 3
    if min_x <= x <= max_x and min_y <= y <= max_y:
        return -min(x - min_x, max_x - x, y - min_y, max_y - y)
    else:
        dx = max(min_x - x, 0, x - max_x)
        dy = max(min_y - y, 0, y - max_y)
        return math.sqrt(dx**2 + dy**2)

# Dubin's car dynamics function
def f(s, u, delta_t):
    x, y, theta = s
    v = 1  # Constant speed
    x_next = x + v * math.cos(theta) * delta_t
    y_next = y + v * math.sin(theta) * delta_t
    theta_next = (theta + u * delta_t) % (2 * math.pi)  # Normalize theta
    return [x_next, y_next, theta_next]

# Discretize actions
def discretize_actions(start, stop, step):
    actions = []
    current = start
    while current <= stop + 1e-9:
        actions.append(round(current, 10))
        current += step
    return actions

# Initialize a grid of x, y, and theta states within a range
def initialize_grid(min_val, max_val, step):
    grid = []
    current = min_val
    while current <= max_val + 1e-9:
        grid.append(round(current, 10))
        current += step
    return grid

# Max-min optimization with dynamic programming
def compute_optimal_policy(time_steps, delta_t, action_step, grid_step):
    # Define grid and actions
    x_grid = initialize_grid(-4, 4, grid_step)
    y_grid = initialize_grid(-3, 3, grid_step)
    theta_grid = initialize_grid(-math.pi/3, math.pi/3, 0.1)  # Discretize theta from 0 to 2*pi
    actions = discretize_actions(-1, 1, action_step)
    
    # Initialize value function for each time step
    V = [{} for _ in range(time_steps + 1)]
    policy = {}  # Store the policy for each state

    # Terminal condition: At the final time step, the value is -g(s)
    for x in x_grid:
        for y in y_grid:
            for theta in theta_grid:
                state = (x, y, theta)
                V[time_steps][state] = -g(state)

    # Backward iteration for optimal policy
    for k in range(time_steps - 1, -1, -1):
        for x in x_grid:
            for y in y_grid:
                for theta in theta_grid:
                    state = (x, y, theta)
                    max_min_value = float('inf')
                    best_action = None

                    for u in actions:
                        next_state = f([x, y, theta], u, delta_t)
                        x_next = round(min(max(next_state[0], -4), 4), 10)
                        y_next = round(min(max(next_state[1], -3), 3), 10)
                        theta_next = round(next_state[2] % (2 * math.pi), 10)  # Keep theta within [0, 2*pi]
                        next_state_rounded = (x_next, y_next, theta_next)

                        # Retrieve future value or assign a high penalty for out of bounds
                        future_value = V[k + 1].get(next_state_rounded, float('inf'))
                        current_value = min(g(state), future_value)
                        

                        if current_value < max_min_value:
                            max_min_value = current_value
                            best_action = u

                    # Update the value function and policy
                    V[k][state] = max_min_value
                    policy[state] = best_action

    return V, policy

# Map state to nearest x, y, theta grid point
def map_to_nearest_grid(state, x_grid, y_grid, theta_grid):
    x, y, theta = state
    x_closest = min(x_grid, key=lambda grid_x: abs(grid_x - x))
    y_closest = min(y_grid, key=lambda grid_y: abs(grid_y - y))
    theta_closest = min(theta_grid, key=lambda grid_theta: abs(grid_theta - theta))
    return (x_closest, y_closest, theta_closest)

# Simulate trajectory following the computed policy
def simulate_trajectory(initial_state, policy, V, time_steps, delta_t, action_step, grid_step):
    x_grid = initialize_grid(-4, 4, grid_step)
    y_grid = initialize_grid(-3, 3, grid_step)
    theta_grid = initialize_grid(-math.pi/3, math.pi/3, 0.1)
    current_state = initial_state
    trajectory = [tuple(current_state)]
    signed_distances = []
    value_over_time = []
    policy_over_time = []

    for k in range(time_steps):
        nearest_state = map_to_nearest_grid(current_state, x_grid, y_grid, theta_grid)
        optimal_action = policy.get(nearest_state, math.pi/3)
        next_state = f(current_state, optimal_action, delta_t)
        
        trajectory.append(tuple(next_state))
        s_distance = g(nearest_state)
        signed_distances.append(s_distance)
        
        value = V[k].get(nearest_state, float('-inf'))
        value_over_time.append(value)
        policy_over_time.append(optimal_action)
        
        current_state = next_state

    return trajectory, value_over_time, signed_distances, policy_over_time

# Parameters for dynamic programming function
time_steps = 40
delta_t = 0.1
action_step = 0.1
grid_step = 0.1  # Increased resolution

# Initial condition
initial_state = [0, 0, 0]

# Compute optimal policy
V, policy = compute_optimal_policy(time_steps, delta_t, action_step, grid_step)

# Simulate the trajectory by following the computed policy
trajectory, value_over_time, signed_distances, policy_over_time = simulate_trajectory(
    initial_state, policy, V, time_steps, delta_t, action_step, grid_step)

# Plot results
trajectory_x = [state[0] for state in trajectory]
trajectory_y = [state[1] for state in trajectory]

min_x, max_x = -4, 4
min_y, max_y = -3, 3

# Plot trajectory
plt.figure(figsize=(16, 4))

# Trajectory plot with boundary, start, and end points
plt.subplot(1, 4, 1)
plt.plot(trajectory_x, trajectory_y, marker='o', label="Trajectory")
plt.plot([min_x, max_x, max_x, min_x, min_x], [min_y, min_y, max_y, max_y, min_y], 'r--', label="Boundary")  # Plot boundary
plt.scatter([trajectory_x[0]], [trajectory_y[0]], color="green", label="Start", zorder=5)  # Start point
plt.scatter([trajectory_x[-1]], [trajectory_y[-1]], color="red", label="End", zorder=5)  # End point
plt.title("Trajectory of Dubin's Car")
plt.xlabel("X Position")
plt.ylabel("Y Position")
plt.legend()
plt.grid(True)

# Value function plot
plt.subplot(1, 4, 2)
plt.plot(range(len(value_over_time)), value_over_time, marker='o')
plt.title("Value Function Over Time")
plt.xlabel("Time Step")
plt.ylabel("Value Function")
plt.grid(True)

# Signed distance plot
plt.subplot(1, 4, 3)
plt.plot(range(len(signed_distances)), signed_distances, marker='o', color='purple')
plt.title("Signed Distance Over Time")
plt.xlabel("Time Step")
plt.ylabel("Signed Distance")
plt.grid(True)

# Policy (action) plot
plt.subplot(1, 4, 4)
plt.plot(range(len(policy_over_time)), policy_over_time, marker='o', color='blue')
plt.title("Optimal Policy Over Time")
plt.xlabel("Time Step")
plt.ylabel("Optimal Action (Control)")
plt.grid(True)

plt.tight_layout()
plt.show()
