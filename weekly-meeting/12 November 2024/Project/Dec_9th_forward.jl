using Plots
gr()
using Distributions

###########################################
# Dynamics and Problem Setup
###########################################

# Dynamics of Dubins's car with disturbance
function dynamics(state, action, disturbance, delta_t)
    x, y, theta = state  
    omega, v = action  # Unpack the action tuple

    # Apply disturbances to each state dimension if desired:
    x_disturbance = disturbance
    y_disturbance = disturbance
    theta_disturbance = disturbance

    x_next = x + (v * cos(theta) + x_disturbance) * delta_t 
    y_next = y + (v * sin(theta) + y_disturbance) * delta_t
    theta_next = theta + (omega + theta_disturbance) * delta_t
    return (x_next, y_next, theta_next)
end

# Signed distance function l(x)
function g(state)
    x, y, theta = state
    min_x, max_x = -1.0, 1.0
    min_y, max_y = -1.0, 1.0
    # For theta, here we don't limit, but you can if desired
    min_theta, max_theta = -Inf, Inf

    if (min_x ≤ x ≤ max_x) && (min_y ≤ y ≤ max_y) && (min_theta ≤ theta ≤ max_theta)
        # Inside M: signed distance >= 0
        dx = min(x - min_x, max_x - x)
        dy = min(y - min_y, max_y - y)
        dtheta = min(theta - min_theta, max_theta - theta)
        distance = min(dx, dy, dtheta)
        signed_distance = distance
    else
        # Outside M: signed distance < 0
        dx = max(min_x - x, 0, x - max_x)
        dy = max(min_y - y, 0, y - max_y)
        if theta < min_theta
            dtheta = min_theta - theta
        elseif theta > max_theta
            dtheta = theta - max_theta
        else
            dtheta = 0
        end
        distance = sqrt(dx^2 + dy^2 + dtheta^2)
        signed_distance = -distance
    end

    return signed_distance
end

# Get closest grid indices
function get_indices(state, x_grid, y_grid, theta_grid)
    x, y, theta = state
    x_idx = argmin(abs.(x_grid .- x))
    y_idx = argmin(abs.(y_grid .- y))
    theta_idx = argmin(abs.(theta_grid .- theta))
    return x_idx, y_idx, theta_idx
end

###########################################
# Plotting Functions
###########################################

function plot_V_heatmap(V, x_grid, y_grid, theta_grid, k, ti)
    # Extract slice of V for specified k and theta index
    V_slice = V[k, :, :, ti]
    V_slice = V_slice'  # Transpose to align axes

    heatmap(
        x_grid,
        y_grid,
        V_slice,
        xlabel = "x",
        ylabel = "y",
        title = "V at k=$k, θ=$(round(theta_grid[ti], digits=2))",
        colorbar = true,
        flip = false
    )
    savefig("V_heatmap_k$(k)_theta$(ti).png")
    display(current())
end

function plot_policy_vector_field(policy, x_grid, y_grid, theta_grid, k, ti, delta_t; scaling_factor=1.0)
    theta = theta_grid[ti]

    num_x = length(x_grid)
    num_y = length(y_grid)
    total_points = num_x * num_y

    X = zeros(total_points)
    Y = zeros(total_points)
    U = zeros(total_points)
    V_vec = zeros(total_points)

    index = 1
    for xi in 1:num_x
        x = x_grid[xi]
        for yi in 1:num_y
            y = y_grid[yi]

            X[index] = x
            Y[index] = y

            action = policy[k, xi, yi, ti]
            omega, v = action

            if isnan(omega) || isnan(v)
                U[index] = 0.0
                V_vec[index] = 0.0
            else
                dx = v * cos(theta) * delta_t
                dy = v * sin(theta) * delta_t

                U[index] = dx * scaling_factor
                V_vec[index] = dy * scaling_factor
            end

            index += 1
        end
    end

    quiver(
        X, Y; quiver = (U, V_vec),
        xlabel = "x",
        ylabel = "y",
        title = "Policy at k=$k, θ=$(round(theta, digits=2))",
        aspect_ratio = :equal,
        legend = false,
        arrow = :closed,
        linewidth = 1.5,
        color = :blue
    )

    boundary_x = [minimum(x_grid), maximum(x_grid), maximum(x_grid), minimum(x_grid), minimum(x_grid)]
    boundary_y = [minimum(y_grid), minimum(y_grid), maximum(y_grid), maximum(y_grid), minimum(y_grid)]
    plot!(boundary_x, boundary_y, lc = :red, lw = 2, label = "Boundary")

    savefig("policy_vector_field_k$(k)_theta$(ti).png")
    display(current())
end

function plot_disturbance_surface(disturbance_map, x_grid, y_grid, k, ti)
    disturbance_slice = disturbance_map[k, :, :, ti]
    disturbance_slice = disturbance_slice'

    surface(
        x_grid, y_grid, disturbance_slice,
        xlabel = "x",
        ylabel = "y",
        zlabel = "Disturbance",
        title = "Disturbance at k=$k, θ index=$ti",
        colorbar = true,
        legend = false
    )

    savefig("disturbance_surface_k$(k)_theta$(ti).png")
    display(current())
end

###########################################
# Problem Parameters and Initialization
###########################################

lower_x, upper_x = -1.0, 1.0
lower_y, upper_y = -1.0, 1.0
lower_omega, upper_omega = -2.0, 2.0
lower_v, upper_v = -1.0, 1.0

num_x, num_y, num_theta = 20, 20, 16
num_omega, num_v = 10, 10

x_grid = range(lower_x, upper_x, length=num_x)
y_grid = range(lower_y, upper_y, length=num_y)
theta_grid = range(-π, π, length=num_theta)
omega_grid = range(lower_omega, upper_omega, length=num_omega)
v_grid = range(lower_v, upper_v, length=num_v)

# Disturbance set (Gaussian-based range)
μ_disturbance = 0.0
σ_disturbance = 0.1
num_disturbance = 5
disturbance_values = range(μ_disturbance - 2σ_disturbance, μ_disturbance + 2σ_disturbance, length=num_disturbance)

time = 5.0
delta_t = 0.1
time_steps = Int(time / delta_t)

V = fill(NaN, time_steps + 1, num_x, num_y, num_theta)
policy = Array{Tuple{Float64, Float64}}(undef, time_steps + 1, num_x, num_y, num_theta)
disturbance_map = fill(NaN, time_steps + 1, num_x, num_y, num_theta)

# Initialize V^0 = l(x)
for xi in 1:num_x
    for yi in 1:num_y
        for ti in 1:num_theta
            state = (x_grid[xi], y_grid[yi], theta_grid[ti])
            V[1, xi, yi, ti] = g(state)
        end
    end
end

###########################################
# Dynamic Programming Iterations
###########################################
#
# V^{k+1}(x) = min { l(x), max_u min_d V^k(x + Δt f(x,u,d)) }
# Start from k=0 given by V[1,...], and compute up to k=time_steps
#
for k in 1:time_steps
    println("Calculating value function at iteration k = $k")
    for xi in 1:num_x
        for yi in 1:num_y
            for ti in 1:num_theta
                state = (x_grid[xi], y_grid[yi], theta_grid[ti])
                l_val = g(state)

                max_u_value = -Inf
                best_action = (NaN, NaN)
                best_disturbance_for_action = NaN

                for omega in omega_grid
                    for v in v_grid
                        action = (omega, v)

                        min_d_value = Inf
                        disturbance_for_this_action = NaN

                        for disturbance in disturbance_values
                            next_state = dynamics(state, action, disturbance, delta_t)
                            x_next_idx, y_next_idx, theta_next_idx = get_indices(next_state, x_grid, y_grid, theta_grid)

                            if 1 ≤ x_next_idx ≤ num_x && 1 ≤ y_next_idx ≤ num_y && 1 ≤ theta_next_idx ≤ num_theta
                                V_next = V[k, x_next_idx, y_next_idx, theta_next_idx]
                            else
                                V_next = -Inf
                            end

                            if V_next < min_d_value
                                min_d_value = V_next
                                disturbance_for_this_action = disturbance
                            end
                        end

                        if min_d_value > max_u_value
                            max_u_value = min_d_value
                            best_action = action
                            best_disturbance_for_action = disturbance_for_this_action
                        end
                    end
                end

                V[k+1, xi, yi, ti] = min(l_val, max_u_value)
                policy[k+1, xi, yi, ti] = best_action
                disturbance_map[k+1, xi, yi, ti] = best_disturbance_for_action
            end
        end
    end
end

###########################################
# Visualization Examples
###########################################
# Plot some slices of the value function and policy
for time_step in [1, time_steps+1]
    # Plot the value function at θ index 1 (arbitrary choice)
    plot_V_heatmap(V, x_grid, y_grid, theta_grid, time_step, 1)
end

# Plot a sample policy vector field at final iteration and θ index 1
plot_policy_vector_field(policy, x_grid, y_grid, theta_grid, time_steps+1, 1, delta_t)

# Plot the disturbance surface at final iteration and θ index 1
plot_disturbance_surface(disturbance_map, x_grid, y_grid, time_steps+1, 1)

###########################################
# Simulation With Value Tracking (Optional)
###########################################

function run_simulation_with_value_tracking(
    initial_state, policy, x_grid, y_grid, theta_grid, V, dynamics, delta_t, time_steps
)
    trajectory = [initial_state]
    inputs = Tuple{Float64, Float64}[]
    value_over_time = Float64[]
    signed_distance_over_time = Float64[]
    disturbance_over_time = Float64[]

    current_state = initial_state

    for k in 1:time_steps
        x, y, theta = current_state

        x_idx = argmin(abs.(x_grid .- x))
        y_idx = argmin(abs.(y_grid .- y))
        theta_idx = argmin(abs.(theta_grid .- theta))

        value_current = V[k, x_idx, y_idx, theta_idx]
        push!(value_over_time, value_current)

        sd_current = g(current_state)
        push!(signed_distance_over_time, sd_current)

        action = policy[k, x_idx, y_idx, theta_idx]
        omega, v = action

        if isnan(omega) || isnan(v)
            println("No optimal action found for state $(current_state) at time step $k. Stopping simulation.")
            break
        end

        # Sample a disturbance
        disturbance = rand(Normal(μ_disturbance, σ_disturbance))
        push!(disturbance_over_time, disturbance)

        next_state = dynamics(current_state, action, disturbance, delta_t)
        x_next, y_next, theta_next = next_state
        # Wrap theta
        theta_next = mod(theta_next + π, 2π) - π
        next_state = (x_next, y_next, theta_next)

        push!(inputs, action)
        push!(trajectory, next_state)

        current_state = next_state
    end

    return trajectory, inputs, value_over_time, signed_distance_over_time, disturbance_over_time
end

# Example simulation
initial_state = (0.5, 0.5, π/2)
trajectory, inputs, value_over_time, signed_distance_over_time, disturbance_over_time = run_simulation_with_value_tracking(
    initial_state, policy, x_grid, y_grid, theta_grid, V, dynamics, delta_t, time_steps
)

# Plot trajectory
trajectory_x = [s[1] for s in trajectory]
trajectory_y = [s[2] for s in trajectory]
time_points = (0:length(inputs)-1) .* delta_t

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
scatter!([trajectory_x[1]], [trajectory_y[1]],
    color = :green, markerstrokecolor = :black, markersize = 8, label = "Initial Position")

boundary_x = [lower_x, upper_x, upper_x, lower_x, lower_x]
boundary_y = [lower_y, lower_y, upper_y, upper_y, lower_y]
plot!(boundary_x, boundary_y, seriestype = :shape, fillalpha=0.0, linecolor=:red, linestyle=:dash, label="Boundary")

savefig("trajectory_plot.png")
display(current())

# Plot control inputs
omega_over_time = [a[1] for a in inputs]
v_over_time = [a[2] for a in inputs]

plot(time_points, omega_over_time,
    xlabel = "Time (s)",
    ylabel = "Angular Velocity ω",
    title = "Control Input ω Over Time",
    label = "ω",
    lw = 2,
    legend = :topright)
savefig("omega_over_time.png")
display(current())

plot(time_points, v_over_time,
    xlabel = "Time (s)",
    ylabel = "Linear Velocity v",
    title = "Control Input v Over Time",
    label = "v",
    lw = 2,
    legend = :topright)
savefig("v_over_time.png")
display(current())

plot(time_points, value_over_time,
    xlabel = "Time (s)",
    ylabel = "Value Function V",
    title = "Value Function Over Time",
    label = "V",
    lw = 2,
    legend = :topright)
savefig("value_function_over_time.png")
display(current())

plot(time_points, signed_distance_over_time,
    xlabel = "Time (s)",
    ylabel = "Signed Distance",
    title = "Signed Distance Over Time",
    label = "Signed Distance",
    lw = 2,
    legend = :topright)
savefig("signed_distance_over_time.png")
display(current())

plot(time_points, disturbance_over_time,
    xlabel = "Time (s)",
    ylabel = "Disturbance",
    title = "Disturbance Over Time",
    label = "Disturbance",
    lw = 2,
    legend = :topright)
savefig("disturbance_over_time.png")
display(current())
