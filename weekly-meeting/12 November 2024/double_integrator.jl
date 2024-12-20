using PlotlyJS, LinearAlgebra, Random, Distributions

# ----------------------------
# 1. Define Dynamics and Cost Functions
# ----------------------------

# Dynamics function for the Dubins car model
function dynamics(state, action, delta_t)
    x, y, theta = state
    v, omega = action
    # Update state based on Dubins car dynamics
    x_next = x + v * cos(theta) * delta_t
    y_next = y + v * sin(theta) * delta_t
    theta_next = theta + omega * delta_t
    # Normalize theta to [0, 2π)
    theta_next = mod(theta_next, 2π)
    return (x_next, y_next, theta_next)
end

# Cost function penalizing proximity to boundaries
function g(state)
    x, y, theta = state
    # Define bounds
    min_x, max_x = -10.0, 10.0
    min_y, max_y = -10.0, 10.0

    # Cost function: penalize being close to boundaries
    if min_x ≤ x ≤ max_x && min_y ≤ y ≤ max_y
        return -minimum([x - min_x, max_x - x, y - min_y, max_y - y])
    else
        dx = max(min_x - x, 0, x - max_x)
        dy = max(min_y - y, 0, y - max_y)
        return sqrt(dx^2 + dy^2)
    end
end

# Function to generate bounded normal samples
function generate_bounded_normal_samples(mean, variance, lower_bound, upper_bound, num_samples)
    std_dev = sqrt(variance)
    samples = Float64[]

    while length(samples) < num_samples
        sample = rand(Normal(mean, std_dev))
        if lower_bound ≤ sample ≤ upper_bound
            push!(samples, sample)
        end
    end

    return samples
end

# Disturbance function adding noise to angular velocity
function disturbance_function()
    # Example disturbance: add noise to angular velocity
    return generate_bounded_normal_samples(0.0, 0.01, -0.05, 0.05, 1)[1]
end

# ----------------------------
# 2. Define Grids and Actions
# ----------------------------

# Define state bounds and grids
lower_x, upper_x = -10.0, 10.0
lower_y, upper_y = -10.0, 10.0
num_x, num_y = 50, 50  # Grid resolution for x and y

x_grid = range(lower_x, upper_x, length=num_x)
y_grid = range(lower_y, upper_y, length=num_y)
Pkg.add("WebIO")
# Convert grids to vectors
x_vector = collect(x_grid)
y_vector = collect(y_grid)

# Create a surface trace
surface_trace = PlotlyJS.surface(
    z = Z,
    x = x_vector,
    y = y_vector
)

# Define the layout
layout = PlotlyJS.Layout(
    title = "3D Surface Plot of V at Time Step 1 (Theta = -π/2)",
    scene = PlotlyJS.attr(
        xaxis = PlotlyJS.attr(title = "State 1 (x)"),
        yaxis = PlotlyJS.attr(title = "State 2 (y)"),
        zaxis = PlotlyJS.attr(title = "Value V")
    )
)

# Generate the plot
plt = PlotlyJS.plot(surface_trace, layout)

# Display the plot
display(plt)
# Define theta grid
num_theta = 50
theta_grid = range(0, stop=2π - 2π/num_theta, length=num_theta)  # Discrete angles

# Define actions (linear and angular velocities)
v_values = [1.0, 2.0, 3.0]        # Linear velocities
omega_values = [-π/4, 0.0, π/4]   # Angular velocities

# Create actions as a vector (1D array)
actions = [(v, omega) for v in v_values for omega in omega_values]
num_actions = length(actions)

time_steps = 1000
delta_t = 0.001  # Time step size

# ----------------------------
# 3. Initialize Value Function and Policy
# ----------------------------

# Initialize Value Function: V[time, theta, y, x]
V = fill(Inf, time_steps + 1, num_theta, num_y, num_x)

# Initialize Policy: policy[time, theta, y, x]
policy = Array{Int}(undef, time_steps, num_theta, num_y, num_x)
policy .= -1  # Initialize with -1 to indicate undefined actions

# Terminal cost is zero
V[end, :, :, :] .= 0.0

# ----------------------------
# 4. Backward Iteration for Value Function and Policy
# ----------------------------

# Function to get the closest grid indices for a given state
function get_indices(state, x_grid, y_grid, theta_grid)
    x, y, theta = state
    x_idx = argmin(abs.(x_grid .- x))
    y_idx = argmin(abs.(y_grid .- y))
    # Find closest theta index
    theta_diff = abs.(theta_grid .- theta)
    theta_idx = argmin(theta_diff)
    return theta_idx, y_idx, x_idx
end

# Perform backward iteration
for k in time_steps:-1:1
    println("Calculating optimal policy for time step $k")
    for (ti, theta) in enumerate(theta_grid)
        for (yi, y) in enumerate(y_grid)
            for (xi, x) in enumerate(x_grid)
                state = (x, y, theta)
                min_value = Inf
                best_action_idx = -1

                for (ai, action) in enumerate(actions)
                    # Apply disturbance to omega
                    v, omega = action
                    omega_disturbed = omega + disturbance_function()
                    action_disturbed = (v, omega_disturbed)

                    # Compute next state
                    next_state = dynamics(state, action_disturbed, delta_t)
                    x_next, y_next, theta_next = next_state

                    # Check if next_state is within bounds
                    if (lower_x ≤ x_next ≤ upper_x) && (lower_y ≤ y_next ≤ upper_y)
                        # Find indices of next_state
                        theta_next_idx, y_next_idx, x_next_idx = get_indices(next_state, x_grid, y_grid, theta_grid)

                        # Ensure indices are within bounds
                        if 1 ≤ theta_next_idx ≤ num_theta && 1 ≤ y_next_idx ≤ num_y && 1 ≤ x_next_idx ≤ num_x
                            current_value = V[k + 1, theta_next_idx, y_next_idx, x_next_idx]
                        else
                            current_value = Inf
                        end
                    else
                        current_value = Inf
                    end

                    # Compute future value
                    future_value = min(g(state), current_value)

                    # Update minimum value and best action
                    if future_value < min_value
                        min_value = future_value
                        best_action_idx = ai
                    end
                end

                # Update Value Function and Policy
                V[k, ti, yi, xi] = min_value
                policy[k, ti, yi, xi] = best_action_idx
            end
        end
    end
end

# ----------------------------
# 5. Prepare Data for Surface Plot
# ----------------------------

# Extract value function slice for theta = -π/2
theta_zero = -π/2
# Find the theta index closest to theta_zero
theta_zero_idx = argmin(abs.(theta_grid .- theta_zero))
current_V = V[1, theta_zero_idx, :, :]  # Value function at time step 1
Z = current_V

# ----------------------------
# 6. Interactive 3D Plot with PlotlyJS
# ----------------------------

# Convert grids to vectors
x_vector = collect(x_grid)
y_vector = collect(y_grid)

# Create a surface trace
surface_trace = PlotlyJS.surface(
    z = Z,
    x = x_vector,
    y = y_vector
)

# Define the layout
layout = PlotlyJS.Layout(
    title = "3D Surface Plot of V at Time Step 1 (Theta = -π/2)",
    scene = PlotlyJS.attr(
        xaxis = PlotlyJS.attr(title = "State 1 (x)"),
        yaxis = PlotlyJS.attr(title = "State 2 (y)"),
        zaxis = PlotlyJS.attr(title = "Value V")
    )
)

# Generate the plot
plt = PlotlyJS.plot(surface_trace, layout)

# Display the plot
display(plt)

# ----------------------------
# 7. Define the Simulation Function
# ----------------------------

# Function to simulate the system using the optimal policy
function simulate_system(initial_state, time_steps, delta_t, V, policy, actions, x_grid, y_grid, theta_grid)
    state_history = Vector{Tuple{Float64, Float64, Float64}}()
    state = initial_state
    push!(state_history, state)

    for t in 1:time_steps
        theta_idx, y_idx, x_idx = get_indices(state, x_grid, y_grid, theta_grid)

        # Ensure indices are within bounds
        if theta_idx > length(theta_grid) || y_idx > length(y_grid) || x_idx > length(x_grid)
            println("Indices out of bounds at time $t. Exiting simulation.")
            break
        end

        best_action_idx = policy[t, theta_idx, y_idx, x_idx]

        # Validate `best_action_idx`
        if best_action_idx < 1 || best_action_idx > length(actions)
            println("Invalid action index at time $t. Exiting simulation.")
            break
        end

        # Retrieve the optimal action
        action = actions[best_action_idx]
        v, omega = action

        # Apply disturbance to omega
        omega_disturbed = omega + disturbance_function()
        action_disturbed = (v, omega_disturbed)

        # Compute the next state
        state = dynamics(state, action_disturbed, delta_t)

        x, y, theta = state
        if !(lower_x ≤ x ≤ upper_x && lower_y ≤ y ≤ upper_y)
            println("State went out of bounds at time $t. Exiting simulation.")
            break
        end

        # Append the new state to history
        push!(state_history, state)
    end

    return state_history
end

# ----------------------------
# 8. Simulate the System
# ----------------------------

# Define initial state
initial_state = (1.0, 2.0, π/4)  # Starting at the origin with θ = π/4

# Simulate the system
state_history = simulate_system(
    initial_state,
    time_steps,
    delta_t,
    V,
    policy,
    actions,
    x_grid,
    y_grid,
    theta_grid
)

# Extract x, y, and theta from the state history
x_history = [s[1] for s in state_history]
y_history = [s[2] for s in state_history]
theta_history = [s[3] for s in state_history]

# ----------------------------
# 9. Plot the Simulation Trajectory
# ----------------------------

# Create a trace for the trajectory
trajectory_trace = PlotlyJS.scatter(
    x = x_history,
    y = y_history,
    mode = "lines+markers",
    name = "Trajectory",
    line = PlotlyJS.attr(width = 2),
    marker = PlotlyJS.attr(size = 8)
)

# Define the layout for the trajectory plot
trajectory_layout = PlotlyJS.Layout(
    title = "Trajectory of Optimal Policy",
    xaxis = PlotlyJS.attr(title = "x"),
    yaxis = PlotlyJS.attr(title = "y"),
    showlegend = true
)

# Generate the trajectory plot
trajectory_plot = PlotlyJS.plot(trajectory_trace, trajectory_layout)

# Display the trajectory plot
display(trajectory_plot)
