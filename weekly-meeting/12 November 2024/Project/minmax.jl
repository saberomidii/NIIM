# Libraries
using Plots
gr()  # Use the GR backend for plotting
using Base.Threads  # For multi-threading

# Dynamics of Dubins's car
function dynamics(state, action, delta_t)
    x, y, theta = state
    omega, v = action  # Unpack the action tuple
    x_next = x + v * cos(theta) * delta_t
    y_next = y + v * sin(theta) * delta_t
    theta_next = theta + omega * delta_t
    return (x_next, y_next, theta_next)
end

# Cost function penalizing proximity to boundaries
function g(state)
    x, y, theta = state
    # Define bounds
    min_x, max_x = -1, 1
    min_y, max_y = -1, 1

    # Cost function: penalize being close to boundaries
    if min_x ≤ x ≤ max_x && min_y ≤ y ≤ max_y
        return -minimum([x - min_x, max_x - x, y - min_y, max_y - y])
    else
        dx = max(min_x - x, 0, x - max_x)
        dy = max(min_y - y, 0, y - max_y)
        return sqrt(dx^2 + dy^2)
    end
end

# Define state bounds and grids
lower_x, upper_x = -1.0, 1.0
lower_y, upper_y = -1.0, 1.0
num_x, num_y = 10, 10  # Grid resolution for x and y
x_grid = round.(collect(range(lower_x, upper_x, length=num_x)), digits=3)
y_grid = round.(collect(range(lower_y, upper_y, length=num_y)), digits=3)

# Define theta grid
# theta_grid = [0, pi/4, pi/2, 3pi/4, pi, 5pi/4, 3pi/2, 7pi/4]
lower_theta, upper_theta = -pi, pi
num_theta = 20
theta_grid = round.(collect(range(lower_theta, upper_theta, length=num_theta)), digits=4)


println(num_theta)
# Define action grids
omega_values = collect(range(-1, 1, step=0.1))
v_values = collect(range(-1, 1, step=0.1))

# Simulation parameters
delta_t = 0.1
num_steps = 25  # Number of simulation steps
time_steps = num_steps  # Match time_steps with num_steps

# Initialize Value Function and Policy
V = fill(Inf, time_steps + 1, num_theta, num_y, num_x)
policy = Array{Tuple{Float64, Float64}}(undef, time_steps, num_theta, num_y, num_x)

# Terminal cost is zero
V[end, :, :, :] .= 0.0

# Function to get the closest grid indices for a given state
function get_indices(state, x_grid, y_grid, theta_grid)
    x, y, theta = state
    x_idx = argmin(abs.(x_grid .- x))
    y_idx = argmin(abs.(y_grid .- y))
    # Handle theta wrapping around 2π
    # theta = mod(theta, 2pi)
    theta_diff = abs.(theta_grid .- theta)
    theta_idx = argmin(theta_diff)
    return theta_idx, y_idx, x_idx
end

# Perform backward iteration to compute Value Function and Policy
for k in time_steps:-1:1
    println("Calculating optimal policy for time step $k")
    for (ti, theta) in enumerate(theta_grid)
        for (yi, y) in enumerate(y_grid)
            for (xi, x) in enumerate(x_grid)
                state = (x, y, theta)
                min_value = Inf
                best_action = (0.0, 0.0)

                for omega in omega_values
                    for v in v_values
                        # Compute the next state
                        next_state = dynamics(state, (omega, v), delta_t)
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

                        # Update minimum value and best actions
                        if future_value < min_value
                            min_value = future_value
                            best_action = (omega, v)
                        end
                    end
                end
                # Update Value Function and Policy
                V[k, ti, yi, xi] = min_value
                policy[k, ti, yi, xi] = best_action
            end
        end
    end
end

println("Value Function and Policy computation completed.")

# Convert grids to vectors for plotting
x_vector = collect(x_grid)
y_vector = collect(y_grid)

# Plot and Save 3D Surface Plot for a Single Theta
theta_idx = 1  # Example: First theta value
current_V = V[1, theta_idx, :, :]  # Value function at the first time step
surface_plot = surface(
    x_vector,
    y_vector,
    current_V',
    xlabel = "State 1 (x)",
    ylabel = "State 2 (y)",
    zlabel = "Value V",
    title = "Value Function for Theta = $(round(theta_grid[theta_idx], digits=2))"
)
savefig(surface_plot, "value_function_theta_$(round(theta_grid[theta_idx], digits=2)).png")
display(surface_plot)

# Define the simulation function
function run_simulation()
    # Initial condition
    initial_state = (0.5, 0.5, pi/2)  # (x, y, theta)
    current_state = initial_state

    # Initialize trajectory and inputs
    trajectory = Vector{Tuple{Float64, Float64, Float64}}()
    push!(trajectory, initial_state)  # Add the initial state
    omegas = Float64[]
    speeds = Float64[]

    # Run simulation
    for t_index = 1:num_steps
        # Decompose the current state
        x, y, theta = current_state

        # Get indices for the current state
        theta_index, y_index, x_index = get_indices(current_state, x_grid, y_grid, theta_grid)

        # Check if indices are within bounds
        # if t_index in 1:size(policy, 1) &&
        #    theta_index in 1:size(policy, 2) &&
        #    y_index in 1:size(policy, 3) &&
        #    x_index in 1:size(policy, 4)
            # Retrieve the action from the policy
            action = policy[t_index, theta_index, y_index, x_index]

            # Extract omega and v from the action and store them
            omega, v = action
            push!(omegas, omega)
            push!(speeds, v)

            # Compute the next state
            current_state = dynamics(current_state, action, delta_t)
            push!(trajectory, current_state)
        # else
        #     println("Indices out of bounds at time step $t_index. Simulation stopped.")
        #     break  # Exit the loop if indices are out of bounds
        # end
    end

    println("Simulation completed.")

    return omegas, speeds, trajectory
end

# Plot Value Function for Different Theta Values
num_theta = length(theta_grid)
plots = []

# for (theta_idx, theta) in enumerate(theta_grid)
#     current_V = V[1, theta_idx, :, :]  # Value function at the first time step
#     p = surface(
#         x_vector,
#         y_vector,
#         current_V',
#         xlabel = "x",
#         ylabel = "y",
#         zlabel = "V",
#         title = "Theta = $(round(theta, digits=2))"
#     )
#     push!(plots, p)
# end

# # Arrange subplots in a grid
# layout = @layout [grid(ceil(Int, sqrt(num_theta)), ceil(Int, sqrt(num_theta)))]

# Combine and display the plots
# combined_plot = plot(plots..., layout = layout, size = (1000, 1000))
# savefig(combined_plot, "value_function_all_thetas.png")
# display(combined_plot)

# Create and Save Animation Over Time for One Theta
theta_idx = 1  # Choose the theta value to animate
animation = @animate for k in 1:time_steps + 1
    current_V = V[k, theta_idx, :, :]
    surface(
        x_vector,
        y_vector,
        current_V',
        xlabel = "x",
        ylabel = "y",
        zlabel = "V",
        title = "Time Step $k, Theta = $(round(theta_grid[theta_idx], digits=2))",
        zlims = (minimum(V), maximum(V))
    )
end
gif(animation, "value_function_animation_theta_$(round(theta_grid[theta_idx], digits=2)).gif", fps = 10)

# Run the simulation
omegas, speeds, trajectory = run_simulation()

# Prepare time vector
input_time = (0:length(omegas)-1) * delta_t  # Length matches omegas and speeds

# Extract trajectory data
x_traj = [s[1] for s in trajectory]
y_traj = [s[2] for s in trajectory]

# Boundary definition
boundary_x = [-1, 1, 1, -1, -1]  # Close the boundary rectangle
boundary_y = [-1, -1, 1, 1, -1]

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

