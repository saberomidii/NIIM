import numpy as np

# Define system parameters
A = np.array([[1.0, 1.0], [0.0, 1.0]])  # State transition matrix
B = np.array([[0.0], [1.0]])            # Control input matrix
Q = np.eye(2)                           # State cost matrix
R = np.array([[1.0]])                   # Control cost matrix
Q_f = np.eye(2) * 10                    # Terminal state cost matrix
x_init = np.array([5.0, 0.0])           # Initial state
u_min, u_max = -2.0, 2.0                # Control limits
N = 10                                  # Time horizon
state_dim = A.shape[0]
control_dim = B.shape[1]
state_grid_size = 0.1                   # Grid size for discretizing states

# Pre-allocate arrays for value function and policy
V = [None] * (N + 1)  # Value function
pi = [None] * N       # Optimal policy

# Discretization function
def discretize_state(x, grid_size=state_grid_size):
    return tuple(np.round(x / grid_size) * grid_size)

# Initialize terminal cost
def terminal_cost(x):
    return x.T @ Q_f @ x

# Value function and policy containers
value_function_table = [{} for _ in range(N + 1)]  # To store value function values

# Terminal cost
V[N] = lambda x: terminal_cost(x)
value_function_table[N] = {discretize_state(x_init): V[N](x_init)}

# Dynamic programming recursion
for k in range(N - 1, -1, -1):
    policies = {}
    def value_function(x):
        x_disc = discretize_state(x)  # Discretize the state
        if x_disc in value_function_table[k + 1]:  # Only evaluate reachable states
            min_cost = float('inf')
            best_u = None
            for u in np.linspace(u_min, u_max, 100):  # Discretize control space
                u = np.array([u])  # Make it a column vector
                x_next = A @ x + B @ u
                x_next_disc = discretize_state(x_next)
                if x_next_disc in value_function_table[k + 1]:
                    cost = x.T @ Q @ x + u.T @ R @ u + value_function_table[k + 1][x_next_disc]
                    if cost < min_cost:
                        min_cost = cost
                        best_u = u
            policies[x_disc] = best_u
            value_function_table[k][x_disc] = min_cost
            return min_cost
        else:
            return float('inf')

    # Evaluate value function for all possible states
    for x_state in value_function_table[k + 1].keys():
        value_function(np.array(x_state))
    pi[k] = policies  # Store policies for this step

# Simulate the system using the optimal policy
x = x_init
trajectory = [x]
controls = []
for k in range(N):
    x_disc = discretize_state(x)
    u_opt = pi[k][x_disc]  # Use discretized state to retrieve optimal control
    x = A @ x + B @ u_opt
    trajectory.append(x)
    controls.append(u_opt)

# Convert trajectory to numpy array for better visualization
trajectory = np.array(trajectory)
controls = np.array(controls)

# Plot results
import matplotlib.pyplot as plt

plt.figure(figsize=(10, 5))
plt.subplot(2, 1, 1)
plt.plot(trajectory[:, 0], label="Position")
plt.plot(trajectory[:, 1], label="Velocity")
plt.xlabel("Time step")
plt.ylabel("State")
plt.legend()
plt.title("State Trajectory")

plt.subplot(2, 1, 2)
plt.step(range(N), controls, where="post")
plt.xlabel("Time step")
plt.ylabel("Control input")
plt.title("Optimal Control Input")

plt.tight_layout()
plt.show()
