# Import packages
using Plots, Base.Threads  # For multi-threading
gr()  # Use the GR backend for plotting

# Dynamics of Dubins's car
function dynamics(state, action, delta_t)
    x, y, theta = state
    omega, v = action  # Unpack the action tuple
    x_next = x + v * cos(theta) * delta_t
    y_next = y + v * sin(theta) * delta_t
    theta_next = theta + omega * delta_t
    return (x_next, y_next, theta_next)
end

# Immediate cost function penalizing proximity to boundaries
function immediate_cost(state)
    x, y, theta = state
    # Define bounds
    min_x, max_x = -1.0, 1.0
    min_y, max_y = -1.0, 1.0

    # Compute minimum distance to the boundary
    if min_x ≤ x ≤ max_x && min_y ≤ y ≤ max_y
        dist_to_boundary = minimum([x - min_x, max_x - x, y - min_y, max_y - y])
        return -dist_to_boundary  # Negative to penalize being close to the boundary
    else
        # If outside the bounds, penalize heavily
        dx = max(min_x - x, 0.0, x - max_x)
        dy = max(min_y - y, 0.0, y - max_y)
        return sqrt(dx^2 + dy^2) * 1000  # Large penalty
    end
end

# Function to get the closest grid indices for a given state
function get_indices(state, x_grid, y_grid, theta_grid)
    x, y, theta = state
    x_idx = argmin(abs.(x_grid .- x))
    y_idx = argmin(abs.(y_grid .- y))
    # Handle theta wrapping around 2π
    theta = mod(theta, 2pi)
    theta_diff = abs.(theta_grid .- theta)
    theta_idx = argmin(theta_diff)
    return x_idx, y_idx, theta_idx
end

# Function to plot heatmap of V for given time step k and theta index ti
function plot_V_heatmap(V, x_grid, y_grid, theta_grid, k, ti)
    # Extract the slice of V for specified k and ti
    V_slice = V[k, :, :, ti]'  # Transpose to match x and y

    # Create the heatmap
    heatmap(
        x_grid,        # x-axis values
        y_grid,        # y-axis values
        V_slice,       # V[yi, xi]
        xlabel = "x",
        ylabel = "y",
        title = "Value Function V at time step $k, θ = $(round(theta_grid[ti], digits=2))",
        colorbar = true,
        flip = false   # Ensure the orientation matches the grid
    )

    # Save and display the plot
    savefig("V_heatmap_k$(k)_theta$(ti).png")
    display(current())
end

# Defining grids
# Define state bounds and grids
lower_x, upper_x = -2.0, 2.0
lower_y, upper_y = -2.0, 2.0
lower_theta, upper_theta = 0, 2π
lower_omega, upper_omega = -1.0, 1.0
lower_v, upper_v = -1.0, 1.0

num_x, num_y, num_theta, num_omega, num_v = 50, 50, 8, 10, 10  # Grid resolution

x_grid = round.(range(lower_x, upper_x, length=num_x), digits=1)
y_grid = round.(range(lower_y, upper_y, length=num_y), digits=1)
theta_grid = range(lower_theta, upper_theta, length=num_theta)

omega_grid = range(lower_omega, upper_omega, length=num_omega)
v_grid = range(lower_v, upper_v, length=num_v)

# DP parameters
time_steps = 5  # Number of time steps
delta_t = 0.1

V = fill(NaN, time_steps + 1, num_x, num_y, num_theta)
policy = Array{Tuple{Float64, Float64}}(undef, time_steps, num_x, num_y, num_theta)

# Initialize terminal cost to zero
for xi in 1:num_x
    for yi in 1:num_y
        for ti in 1:num_theta
            V[time_steps + 1, xi, yi, ti] = 0.0
        end
    end
end

# Perform backward iteration to compute Value Function and Policy
for k in time_steps:-1:1
    println("Calculating optimal policy for time step $k")
    for xi in 1:num_x
        x = x_grid[xi]
        for yi in 1:num_y
            y = y_grid[yi]
            for ti in 1:num_theta
                theta = theta_grid[ti]
                state = (x, y, theta)
                min_value = Inf
                best_action = (NaN, NaN)
                
                # Compute immediate cost
                cost = immediate_cost(state)
                
                for omega in omega_grid
                    for v in v_grid
                        # Compute the next state
                        next_state = dynamics(state, (omega, v), delta_t)
                        x_next, y_next, theta_next = next_state

                        # Check if next_state is within bounds
                        if (lower_x ≤ x_next ≤ upper_x) && (lower_y ≤ y_next ≤ upper_y)
                            # Find indices of next_state
                            x_next_idx, y_next_idx, theta_next_idx = get_indices(next_state, x_grid, y_grid, theta_grid)

                            # Ensure indices are within bounds
                            if 1 ≤ x_next_idx ≤ num_x && 1 ≤ y_next_idx ≤ num_y && 1 ≤ theta_next_idx ≤ num_theta
                                current_value = V[k + 1, x_next_idx, y_next_idx, theta_next_idx]
                            else
                                current_value = Inf
                            end
                        else
                            current_value = Inf
                        end

                        # Compute future value using Bellman equation
                        future_value = cost + current_value

                        # Update minimum value and best action
                        if future_value < min_value
                            min_value = future_value
                            best_action = (omega, v)
                        end
                    end
                end

                # Update Value Function and Policy
                V[k, xi, yi, ti] = min_value
                policy[k, xi, yi, ti] = best_action
            end
        end
    end
end

# Plotting Data
k = 1  # Choose the time step index

for ti in 1:length(theta_grid)
    plot_V_heatmap(V, x_grid, y_grid, theta_grid, k, ti)
end

## Simulation 
# Initial condition
initial_state = (0.5, 0.5, 0)  # (x, y, theta)
update_state = initial_state

# Initialize trajectory and inputs
trajectory = Vector{Tuple{Float64, Float64, Float64}}()
push!(trajectory, initial_state)  # Add the initial state
omegas = Float64[]
speeds = Float64[]

# Run simulation
for t_index = 1:time_steps
    x, y, theta = update_state

    # Get indices for the current state
    x_index, y_index, theta_index = get_indices(update_state, x_grid, y_grid, theta_grid)
    action = policy[t_index, x_index, y_index, theta_index]

    # Extract omega and v from the action and store them
    omega, v = action
    push!(omegas, omega)
    push!(speeds, v)

    # Compute the next state
    update_state = dynamics(update_state, action, delta_t)
    push!(trajectory, update_state)
end
println("Simulation completed.")

# Prepare time vector
input_time = (0:length(omegas)-1) * delta_t  # Length matches omegas and speeds

# Extract trajectory data
x_traj = [s[1] for s in trajectory]
y_traj = [s[2] for s in trajectory]

# Boundary definition
boundary_x = [-1, 1, 1, -1, -1]  # Close the boundary rectangle
boundary_y = [-1, -1, 1, 1, -1]

# Plotting (optional)
# 1. Plot omega (input) over time
omega_plot = plot(input_time, omegas, 
    xlabel = "Time (s)", 
    ylabel = "Omega (rad/s)", 
    title = "Omega Over Time", 
    label = "Omega", 
    color = :blue)

# 2. Plot constant speed v over time
speed_plot = plot(input_time, speeds, 
    xlabel = "Time (s)", 
    ylabel = "Speed (units/s)", 
    title = "Speed Over Time", 
    label = "Speed", 
    color = :green)

# 3. Plot trajectory with boundary and initial condition
trajectory_plot = plot(x_traj, y_traj, 
    label = "Trajectory", 
    xlabel = "x", ylabel = "y", 
    title = "Trajectory with Boundary", 
    legend = true, aspect_ratio=:equal)

# Add boundary
plot!(trajectory_plot, boundary_x, boundary_y, color=:red, label="Boundary", linewidth=2)

# Mark initial condition
scatter!(trajectory_plot, [x_traj[1]], [y_traj[1]], color=:black, label="Initial Condition", markersize=8)

# Combine all three plots into a single layout
combined_plot = plot(trajectory_plot, omega_plot, speed_plot, layout=(3, 1), size=(800, 1200))

# Save the plot and display
savefig(combined_plot, "trajectory_inputs_speeds_plot.png")
display(combined_plot)
