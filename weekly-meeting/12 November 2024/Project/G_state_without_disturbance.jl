# Import necessary packages
using Plots
gr()  # Use the GR backend for plotting
using Distributions  # For normal distribution

# Dynamics of Dubins's car with disturbance
function dynamics(state, action, disturbance, delta_t)
    x, y, theta = state  
    omega, v = action  # Unpack the action tuple
    
    # Apply disturbance (e.g., additive disturbance to x and y)
    x_disturbance = disturbance
    y_disturbance = disturbance
    theta_disturbance = disturbance  # Assuming disturbance in theta
    
    # Update state with dynamics and disturbance
    x_next = x + (v * cos(theta) + x_disturbance) * delta_t 
    y_next = y + (v * sin(theta) + y_disturbance) * delta_t
    theta_next = theta + (omega + theta_disturbance) * delta_t
    return (x_next, y_next, theta_next)
end

function g(state)
    x, y, theta = state
    # Define bounds for x and y
    min_x, max_x = -1.0, 1.0
    min_y, max_y = -1.0, 1.0
    # Define bounds for theta
    min_theta, max_theta = -Inf, Inf  # Example bounds; adjust as needed

    # Check if the point is inside the set M (the rectangular prism in x, y, theta space)
    if min_x ≤ x ≤ max_x && min_y ≤ y ≤ max_y && min_theta ≤ theta ≤ max_theta
        # Point is inside M
        # Compute s_M(z) = -inf_{y∈ℝ^m \ M} |z - y|
        # Distance to the closest point outside M (boundary)
        dx = min(x - min_x, max_x - x)
        dy = min(y - min_y, max_y - y)
        dtheta = min(theta - min_theta, max_theta - theta)
        distance = min(dx, dy, dtheta)
        signed_distance = distance
    else
        # Point is outside M
        # Compute s_M(z) = inf_{y∈M} |z - y|
        # Distance to the closest point inside M (boundary)
        dx = max(min_x - x, 0, x - max_x)
        dy = max(min_y - y, 0, y - max_y)
        # Handle periodicity of theta if applicable
        if theta < min_theta
            dtheta = min_theta - theta
        elseif theta > max_theta
            dtheta = theta - max_theta
        else
            dtheta = 0  # Inside theta bounds
        end
        # Total Euclidean distance in 3D space
        distance = sqrt(dx^2 + dy^2 + dtheta^2)
        signed_distance = -distance
    end

    return signed_distance
end

# Function to get the closest grid indices for a given state
function get_indices(state, x_grid, y_grid, theta_grid)
    x, y, theta = state
    x_idx = argmin(abs.(x_grid .- x))
    y_idx = argmin(abs.(y_grid .- y))
    # Handle theta wrapping around 2π
    theta_diff = abs.(theta_grid .- theta)
    theta_idx = argmin(theta_diff)
    return x_idx, y_idx, theta_idx
end

# Function to plot heatmap of V for given time step k and theta index ti
function plot_V_heatmap(V, x_grid, y_grid, theta_grid, k, ti)
    # Extract the slice of V for specified k and ti
    V_slice = V[k, :, :, ti]  # V_slice[xi, yi]

    # Transpose V_slice to match x and y axes
    V_slice = V_slice'

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

function plot_policy_vector_field(policy, x_grid, y_grid, theta_grid, k, ti, delta_t; scaling_factor=1.0)
    # Extract the theta value
    theta = theta_grid[ti]
    
    num_x = length(x_grid)
    num_y = length(y_grid)
    total_points = num_x * num_y
    
    # Initialize arrays to hold positions and vector components
    X = zeros(total_points)
    Y = zeros(total_points)
    U = zeros(total_points)
    V_vec = zeros(total_points)
    
    index = 1  # Index for the flattened arrays
    
    for xi in 1:num_x
        x = x_grid[xi]
        for yi in 1:num_y
            y = y_grid[yi]
                        
            # Store the positions
            X[index] = x
            Y[index] = y
                        
            # Extract the action from the policy
            action = policy[k, xi, yi, ti]
            omega, v = action
                        
            # Handle undefined or NaN actions
            if isnan(omega) || isnan(v)
                U[index] = 0.0
                V_vec[index] = 0.0
            else
                # Compute the vector components based on the action and theta
                dx = v * cos(theta) * delta_t
                dy = v * sin(theta) * delta_t
                            
                # Apply scaling factor to make arrows larger
                U[index] = dx * scaling_factor
                V_vec[index] = dy * scaling_factor
            end
                        
            index += 1
        end
    end
    
    # Create the quiver plot with correct argument placement
    quiver(
        X, Y;
        quiver = (U, V_vec),
        xlabel = "x",
        ylabel = "y",
        title = "Policy Vector Field at time step $k, θ = $(round(theta, digits=2))",
        aspect_ratio = :equal,
        legend = false,
        arrow = :closed,       # Use closed arrowheads
        linewidth = 1.5,       # Increase the linewidth of arrows
        color = :blue,         # Set arrow color
        quiverkey = false
    )
    
    # Add boundary lines
    boundary_x = [minimum(x_grid), maximum(x_grid), maximum(x_grid), minimum(x_grid), minimum(x_grid)]
    boundary_y = [minimum(y_grid), minimum(y_grid), maximum(y_grid), maximum(y_grid), minimum(y_grid)]
    plot!(boundary_x, boundary_y, lc = :red, lw = 2, label = "Boundary")
    
    # Save and display the plot
    savefig("policy_vector_field_k$(k)_theta$(ti).png")
    display(current())
end

# Function to plot disturbance over x-y plane at a given time step and theta index
function plot_disturbance_surface(disturbance_map, x_grid, y_grid, k, ti)
    disturbance_slice = disturbance_map[k, :, :, ti]  # Extract disturbance values at time k and theta index ti
    disturbance_slice = disturbance_slice'  # Transpose to match x and y axes

    surface(
        x_grid, y_grid, disturbance_slice,
        xlabel = "x",
        ylabel = "y",
        zlabel = "Disturbance",
        title = "Disturbance over x-y Plane at time step $k, θ index $ti",
        colorbar = true,
        legend = false
    )

    # Save and display the plot
    savefig("disturbance_surface_k$(k)_theta$(ti).png")
    display(current())
end

##### Defining grids
# Define state bounds and grids
lower_x, upper_x = -1.0, 1.0
lower_y, upper_y = -1.0, 1.0
lower_omega, upper_omega = -2, 2
lower_v, upper_v = -1, 1

num_x, num_y, num_theta = 20, 20, 16  # Grid resolution for x, y, and theta
num_omega, num_v = 10, 10  # Grid resolution for omega and v

x_grid = range(lower_x, upper_x, length=num_x)
y_grid = range(lower_y, upper_y, length=num_y)
theta_grid = range(-π, π, length=num_theta)
omega_grid = range(lower_omega, upper_omega, length=num_omega)
v_grid = range(lower_v, upper_v, length=num_v)

# Define disturbance as a normal distribution with mean μ and standard deviation σ
μ_disturbance = 0  # Mean of disturbance
σ_disturbance = 0 # Standard deviation of disturbance
num_disturbance = 0  # Number of disturbance values to consider in the grid
disturbance_values = range(μ_disturbance * σ_disturbance, μ_disturbance + num_disturbance * σ_disturbance, length=num_disturbance)
# Calculate the probability density for visualization
disturbance_pdf = pdf.(Normal(μ_disturbance, σ_disturbance), disturbance_values)
# Normalize disturbance_pdf if needed

# Initialize the value function
time = 5
delta_t = 0.1
time_steps = Int(time / delta_t)
V = fill(NaN, time_steps + 1, num_x, num_y, num_theta)
policy = Array{Tuple{Float64, Float64}}(undef, time_steps, num_x, num_y, num_theta)

# Initialize terminal cost
for xi in 1:num_x
    for yi in 1:num_y
        for ti in 1:num_theta
            state = (x_grid[xi], y_grid[yi], theta_grid[ti])
            V[time_steps + 1, xi, yi, ti] = g(state)
        end
    end
end
# Initialize disturbance_map to store the disturbance that minimizes the max future value
disturbance_map = fill(NaN, time_steps, num_x, num_y, num_theta)

# Perform backward iteration to compute Value Function, Policy, and Disturbance Map
for k in time_steps:-1:1
    println("Calculating optimal policy for time step $k")
    for (xi, x) in enumerate(x_grid)
        for (yi, y) in enumerate(y_grid)
            for (ti, theta) in enumerate(theta_grid)
                state = (x, y, theta)
                max_value = -Inf  # Initialization for maximization over actions
                best_action = (NaN, NaN)
                best_disturbance = NaN  # Initialize best disturbance for the best action

                # Loop over control actions (maximization)
                for omega in omega_grid
                    for v in v_grid
                        action = (omega, v)
                        
                        min_future_value = Inf  # Initialization for minimization over disturbances
                        disturbance_for_current_action = NaN

                        # Loop over disturbances (minimization)
                        for disturbance in disturbance_values
                            # Compute the next state with disturbance
                            next_state = dynamics(state, action, disturbance, delta_t)
                            x_next, y_next, theta_next = next_state
                            
                            # Get indices of the next state
                            x_next_idx, y_next_idx, theta_next_idx = get_indices(next_state, x_grid, y_grid, theta_grid)
                            
                            # Check if indices are within bounds
                            if 1 ≤ x_next_idx ≤ num_x && 1 ≤ y_next_idx ≤ num_y && 1 ≤ theta_next_idx ≤ num_theta
                                # Retrieve the Value Function at the next state
                                V_next = V[k + 1, x_next_idx, y_next_idx, theta_next_idx]
                            else
                                V_next = -Inf  # Assign a penalty for out-of-bounds transitions
                            end
                            
                            # Compute signed distance at the next state
                            sign_distance = g(state)
                            
                            # Compute future_value as the minimum between sign_distance and V_next
                            future_value = min(sign_distance, V_next)
                            
                            # Update min_future_value and disturbance_for_current_action if future_value is less
                            if future_value < min_future_value
                                min_future_value = future_value
                                disturbance_for_current_action = disturbance
                            end
                        end
                        
                        # Update max_value, best_action, and best_disturbance if min_future_value is greater
                        if min_future_value > max_value
                            max_value = min_future_value
                            best_action = action
                            best_disturbance = disturbance_for_current_action
                        end
                    end
                end
                
                # Update Value Function, Policy, and Disturbance Map
                V[k, xi, yi, ti] = max_value
                policy[k, xi, yi, ti] = best_action
                disturbance_map[k, xi, yi, ti] = best_disturbance  # Store the disturbance
            end
        end
    end
end
# Example usage: Plot disturbance at time step 1 and theta index 1
plot_disturbance_surface(disturbance_map, x_grid, y_grid, 1, 1)


# Plotting Data
for time in 1:5:time_steps
    plot_V_heatmap(V, x_grid, y_grid, theta_grid, time, 1)
end

# Plotting Data
for ti in 1:length(theta_grid)
    plot_V_heatmap(V, x_grid, y_grid, theta_grid, 1, ti)
end

for ti in 1:5:length(theta_grid)
    plot_policy_vector_field(policy, x_grid, y_grid, theta_grid, 1, ti, delta_t)
end

# Simulation Function with Disturbance Tracking
function run_simulation_with_value_tracking(
    initial_state, policy, x_grid, y_grid, theta_grid, V, dynamics, delta_t, time_steps
)
    trajectory = [initial_state]
    inputs = Tuple{Float64, Float64}[]
    value_over_time = Float64[]
    signed_distance_over_time = Float64[]
    disturbance_over_time = Float64[]  # Track disturbances

    current_state = initial_state

    for k in 1:time_steps
        x, y, theta = current_state

        # Find the closest grid indices
        x_idx = argmin(abs.(x_grid .- x))
        y_idx = argmin(abs.(y_grid .- y))
        theta_idx = argmin(abs.(theta_grid .- theta))

        # Get the value of the current state from the value function
        value_current = V[k, x_idx, y_idx, theta_idx]
        push!(value_over_time, value_current)

        # Compute the signed distance at the current state
        signed_distance_current = g(current_state)
        push!(signed_distance_over_time, signed_distance_current)

        # Get the optimal action
        action = policy[k, x_idx, y_idx, theta_idx]
        omega, v = action

        # Check for NaN actions
        if isnan(omega) || isnan(v)
            println("No optimal action found for state ($x, $y, $theta) at time step $k. Stopping simulation.")
            break
        end

        # Sample disturbance from normal distribution
        disturbance = rand(Normal(μ_disturbance, σ_disturbance))
        # Save the disturbance used
        push!(disturbance_over_time, disturbance)

        # Apply dynamics with sampled disturbance to get next state
        next_state = dynamics(current_state, action, disturbance, delta_t)
        x_next, y_next, theta_next = next_state

        # Wrap theta_next to [-π, π]
        theta_next = mod(theta_next + π, 2π) - π
        next_state = (x_next, y_next, theta_next)  # Create a new tuple with wrapped theta

        # Append the action and next state to the lists
        push!(inputs, action)
        push!(trajectory, next_state)

        current_state = next_state
    end

    return trajectory, inputs, value_over_time, signed_distance_over_time, disturbance_over_time
end

# Set the initial state
initial_state = (0.5, 0.5, π/2)

# Run the simulation
trajectory, inputs, value_over_time, signed_distance_over_time, disturbance_over_time = run_simulation_with_value_tracking(
    initial_state, policy, x_grid, y_grid, theta_grid, V, dynamics, delta_t, time_steps
)

# Extract positions and time points
trajectory_x = [state[1] for state in trajectory]
trajectory_y = [state[2] for state in trajectory]
time_points = (0:length(inputs)-1) .* delta_t

# Plot the trajectory
plot(
    trajectory_x, trajectory_y,
    seriestype = :path,
    xlabel = "x",
    ylabel = "y",
    title = "Simulation Trajectory",
    label = "Trajectory",
    lw = 2,
    legend = :topright
)

# Mark the initial point
scatter!(
    [trajectory_x[1]], [trajectory_y[1]],
    color = :green,
    markerstrokecolor = :black,
    markersize = 8,
    label = "Initial Position"
)

# Add boundary lines
boundary_x = [lower_x, upper_x, upper_x, lower_x, lower_x]
boundary_y = [lower_y, lower_y, upper_y, upper_y, lower_y]
plot!(
    boundary_x, boundary_y,
    seriestype = :shape,
    fillalpha = 0.0,
    linecolor = :red,
    linestyle = :dash,
    label = "Boundary"
)

savefig("trajectory_plot.png")
display(current())

# Plot omega over time
omega_over_time = [action[1] for action in inputs]
plot(
    time_points, omega_over_time,
    xlabel = "Time (s)",
    ylabel = "Angular Velocity ω",
    title = "Control Input ω Over Time",
    label = "ω",
    lw = 2,
    legend = :topright
)

savefig("omega_over_time.png")
display(current())

# Plot v over time
v_over_time = [action[2] for action in inputs]
plot(
    time_points, v_over_time,
    xlabel = "Time (s)",
    ylabel = "Linear Velocity v",
    title = "Control Input v Over Time",
    label = "v",
    lw = 2,
    legend = :topright
)

savefig("v_over_time.png")
display(current())

# Plot value function over time
plot(
    time_points, value_over_time,
    xlabel = "Time (s)",
    ylabel = "Value Function V",
    title = "Value Function Over Time",
    label = "V",
    lw = 2,
    legend = :topright
)

savefig("value_function_over_time.png")
display(current())

# Plot signed distance over time
plot(
    time_points, signed_distance_over_time,
    xlabel = "Time (s)",
    ylabel = "Signed Distance",
    title = "Signed Distance Function Over Time",
    label = "Signed Distance",
    lw = 2,
    legend = :topright
)

savefig("signed_distance_over_time.png")
display(current())

# Plot disturbance over time
plot(
    time_points, disturbance_over_time,
    xlabel = "Time (s)",
    ylabel = "Disturbance",
    title = "Disturbance Over Time",
    label = "Disturbance",
    lw = 2,
    legend = :topright
)

savefig("disturbance_over_time.png")
display(current())


