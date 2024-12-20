import numpy as np
import matplotlib.pyplot as plt

# Define the Dubin's car dynamics
def dubins_car(state, input_omega, adversial_disturbance, v, dt):
    x, y, theta = state
    x_next = x + v * np.cos(theta) * dt
    y_next = y + v * np.sin(theta) * dt
    theta_next = theta + (input_omega + adversial_disturbance) * dt
    return np.array([x_next, y_next, theta_next])

# Define the signed distance function
def signed_distance(position, min_vals, max_vals):
    x, y = position
    inside_x = min_vals[0] <= x <= max_vals[0]
    inside_y = min_vals[1] <= y <= max_vals[1]

    if inside_x and inside_y:
        distance_to_min_x = x - min_vals[0]
        distance_to_max_x = max_vals[0] - x
        distance_to_min_y = y - min_vals[1]
        distance_to_max_y = max_vals[1] - y
        min_distance = min(distance_to_min_x, distance_to_max_x,
                           distance_to_min_y, distance_to_max_y)
        return -min_distance
    else:
        dx = max(min_vals[0] - x, 0, x - max_vals[0])
        dy = max(min_vals[1] - y, 0, y - max_vals[1])
        return np.sqrt(dx**2 + dy**2)

# Dynamic Programming to maximize the value function
def dynamic_programming(initial_state, min_vals, max_vals, v, dt, time_steps, actions, adversial_disturbance=0.0):
    # Initialize value function dictionary
    value_function = {initial_state.tobytes(): 0}

    for t in range(time_steps):
        new_value_function = {}
        for state_bytes, value in value_function.items():
            state = np.frombuffer(state_bytes, dtype=np.float64)
            for action in actions:
                next_state = dubins_car(state, action, adversial_disturbance, v, dt)
                position = next_state[:2]
                s_distance = signed_distance(position, min_vals, max_vals)
                # Update the value function with the maximum signed distance
                state_key = next_state.tobytes()
                if state_key not in new_value_function or s_distance > new_value_function[state_key]:
                    new_value_function[state_key] = s_distance
        value_function = new_value_function
    return value_function

# Function to plot the value function
def plot_value_function(value_function, title="Value Function", min_vals=None, max_vals=None):
    x_vals = []
    y_vals = []
    s_distances = []

    for state_bytes, s_distance in value_function.items():
        state = np.frombuffer(state_bytes, dtype=np.float64)
        x, y = state[:2]
        x_vals.append(x)
        y_vals.append(y)
        s_distances.append(s_distance)

    plt.figure(figsize=(8, 6))
    scatter = plt.scatter(x_vals, y_vals, c=s_distances, cmap='viridis', marker='o')
    plt.colorbar(scatter, label='Signed Distance')
    plt.xlabel('X Position')
    plt.ylabel('Y Position')
    plt.title(title)

    if min_vals is not None and max_vals is not None:
        # Draw the rectangle representing the target set
        rectangle = plt.Rectangle((min_vals[0], min_vals[1]),
                                  max_vals[0] - min_vals[0],
                                  max_vals[1] - min_vals[1],
                                  linewidth=1, edgecolor='red', facecolor='none')
        plt.gca().add_patch(rectangle)
        plt.text(min_vals[0], max_vals[1] + 0.5, 'Target Set', color='red')

    plt.grid(True)
    plt.show()
