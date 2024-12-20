import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math

# ----------------------------
# 1. Define Dynamics and Cost Functions
# ----------------------------

def dynamics(state, action, delta_t):
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
    v_next = v + action * delta_t
    return (x_next, v_next)

def g(state):
    """
    Defines the cost function or signed distance function.
    
    Parameters:
    - state (tuple): Current state as (x, v).
    
    Returns:
    - float: Cost associated with the state.
    """
    x, v = state  # Corrected from y to v
    min_x, max_x = -1, 1
    min_v, max_v = -0.2, 0.2
    if min_x <= x <= max_x and min_v <= v <= max_v:
        return -min(x - min_x, max_x - x, v - min_v, max_v - v)
    else:
        dx = max(min_x - x, 0, x - max_x)
        dv = max(min_v - v, 0, v - max_v)
        return math.sqrt(dx**2 + dv**2)

# ----------------------------
# 2. Define Grids and Actions
# ----------------------------

# Define the grid parameters
lower_value = [-1.1, -0.25, -0.1]  # [x_min, v_min, action_min]
upper_value = [1.1, 0.25, 0.1]      # [x_max, v_max, action_max]
num_points = [100, 200, 100]       # [x_points, v_points, action_points]

# Generate grids using linspace
x_grid = np.linspace(lower_value[0], upper_value[0], num=num_points[0])
v_grid = np.linspace(lower_value[1], upper_value[1], num=num_points[1])
actions = np.linspace(lower_value[2], upper_value[2], num=num_points[2])

# Define time parameters
time_steps = 10
delta_t = 0.1

# ----------------------------
# 3. Initialize Value Function
# ----------------------------

# Initialize value function for each time step as a list of dictionaries
V = [{} for _ in range(time_steps + 1)]

# Terminal condition: At the final time step, the value is 0
for x in x_grid:
    for v in v_grid:
        state = (x, v)
        V[time_steps][state] = 0

# ----------------------------
# 4. Backward Iteration for Value Function
# ----------------------------

for k in range(time_steps - 1, -1, -1):
    print(f"Processing time step {k}")
    for x in x_grid:
        for v in v_grid:
            state = (x, v)  # Corrected from (x, y) to (x, v)
            max_min_value = float('-inf')
            best_action = None

            for u in actions:
                next_state = dynamics(state, u, delta_t)

                # Retrieve future value or assign a high penalty for out of bounds
                future_value = V[k + 1].get(next_state, float('inf'))
                current_value = min(-g(state), future_value)

                if current_value > max_min_value:
                    max_min_value = current_value
                    best_action = u

            # Update the value function
            V[k][state] = max_min_value

# ----------------------------
# 5. Prepare Data for Surface Plot
# ----------------------------

# Access the value function at time=0
current_V = V[0]

# Create meshgrid for X and V
X_mesh, Y_mesh = np.meshgrid(x_grid, v_grid)  # Y_mesh corresponds to v_grid

# Populate Z with corresponding V values
# Using list comprehension for efficiency
# Note: Ensuring that Z has the shape (len(v_grid), len(x_grid)) == (200, 100)
Z = np.array([[current_V.get((x, v), np.nan) for x in x_grid] for v in v_grid])

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
# No color bar as per your request

# ----------------------------
# Subplot 2: V vs. X Plot
# ----------------------------
ax2 = fig.add_subplot(2, 2, 2)
# Select a fixed v, e.g., median v
fixed_v = v_grid[len(v_grid) // 2]
V_vs_X = [current_V.get((x, fixed_v), np.nan) for x in x_grid]
ax2.plot(x_grid, V_vs_X, color='blue')
ax2.set_xlabel('State 1 (x)')
ax2.set_ylabel('Value V')
ax2.set_title(f'V vs. X at v = {fixed_v:.2f}')
ax2.grid(True)

# ----------------------------
# Subplot 3: V vs. V Plot
# ----------------------------
ax3 = fig.add_subplot(2, 2, 3)
# Select a fixed x, e.g., median x
fixed_x = x_grid[len(x_grid) // 2]
V_vs_V = [current_V.get((fixed_x, v), np.nan) for v in v_grid]
ax3.plot(v_grid, V_vs_V, color='green')
ax3.set_xlabel('State 2 (v)')
ax3.set_ylabel('Value V')
ax3.set_title(f'V vs. v at x = {fixed_x:.2f}')
ax3.grid(True)

# ----------------------------
# Subplot 4: Top View with Contour
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

# Adjust layout for better spacing
plt.tight_layout()

# Show all plots
plt.show()
