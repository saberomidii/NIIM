# Import the Plots library
using Plots

# Define the cost function or signed distance function
function g(state)
    """
    Defines the cost function or signed distance function.
    
    Parameters:
    - state (Tuple): Current state as (x, v).
    
    Returns:
    - Float64: Cost associated with the state.
    """
    x, v = state
    min_x, max_x = -1, 1
    min_v, max_v = -0.2, 0.2

    if min_x <= x <= max_x && min_v <= v <= max_v
        # Inside the boundary: positive distance to the closest boundary
        return -minimum([x - min_x, max_x - x, v - min_v, max_v - v])
    else
        # Outside the boundary: Euclidean distance to the boundary
        dx = max(min_x - x, 0, x - max_x)
        dv = max(min_v - v, 0, v - max_v)
        return sqrt(dx^2 + dv^2)
    end
end

# Define the Dubins car dynamics
function dubins_dynamics(state, control, v, delta_t)
    """
    Computes the next state for a Dubins car model.

    Parameters:
    - state (Tuple): Current state as (x, y, θ).
    - control (Float64): Control input (ω - turning rate).
    - v (Float64): Linear velocity.
    - delta_t (Float64): Time increment.

    Returns:
    - Tuple: Next state as (x_next, y_next, θ_next).
    """
    x, y, θ = state
    ω = control

    # Update the state
    x_next = x + v * cos(θ) * delta_t
    y_next = y + v * sin(θ) * delta_t
    θ_next = θ + ω * delta_t

    # Ensure θ is within [-π, π]
    θ_next = mod(θ_next + π, 2π) - π

    return (x_next, y_next, θ_next)
end

# Initialize parameters
delta_t = 0.01
value_function_data = Dict()  # Dictionary to store value function results

# Define grid bounds
lower_value = [-1.1, -0.25, -π, -1]  # [x_min, y_min, θ_min, control_min]
upper_value = [1.1, 0.25, π, 1]      # [x_max, y_max, θ_max, control_max]
num_points = [20, 20, 20, 20]        # [x_points, y_points, θ_points, control_points]

# Compute grid steps
grid_step = [(upper_value[i] - lower_value[i]) / (num_points[i] - 1) for i in 1:length(lower_value)]

# Loop through grid points and compute the value function
for time in 20:-1:1  # Time dimension (outer loop)
    for i_x in 1:num_points[1]  # x dimension
        for i_y in 1:num_points[2]  # y dimension
            for i_theta in 1:num_points[3]  # θ dimension
                for i_u in 1:num_points[4]  # Control dimension
                    # Compute current state and control
                    x = lower_value[1] + (i_x - 1) * grid_step[1]
                    y = lower_value[2] + (i_y - 1) * grid_step[2]
                    θ = lower_value[3] + (i_theta - 1) * grid_step[3]
                    u = lower_value[4] + (i_u - 1) * grid_step[4]

                    state = (x, y, θ)

                    # Compute next state and value function
                    next_state = dubins_dynamics(state, u, 1.0, delta_t)  # Assuming v = 1.0
                    vf = min(g((x, y)), u)  # Replace `g` with appropriate cost function

                    # finding the input that maximize the minimum
                    


                    # Save result to dictionary
                    value_function_data[(time, x, y, θ, u)] = vf
                end
            end
        end
    end
end

println("Done!")
println("Value function for (time=20, x=-1.1, y=-0.25, θ=-π, u=-1): ",
        value_function_data[(20, -1.1, -0.25, -π, -1)])
