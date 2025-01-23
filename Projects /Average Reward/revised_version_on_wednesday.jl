##########################################################
# ENTIRE CODE
##########################################################
using Distributions
using StatsFuns
using Statistics
using Random
using NearestNeighbors
using JuMP
import Gurobi
using GLMakie

##########################################################
# 1) Setup: parameters, discretization, etc.
##########################################################
ENV["GRB_LICENSE_FILE"] = "/Users/saber/gurobi/gurobi.lic"

# Optional: set a random seed if desired
Random.seed!(1234)

# Noise standard deviations, pick one for demonstration
const sigma_list  = [0.08]

# Example center_list for alpha constraints (not used in detail here)
const center_list = [(0.0, 0.0)]

# Number of states in each dimension
const num_points_state = 51
positions  = collect(LinRange(-2, 2, num_points_state))   # possible x-values
velocities = collect(LinRange(-2, 2, num_points_state))   # possible v-values

# Cartesian product of (x, v) forms the entire state space
const states_2d  = [(x, v) for x in positions for v in velocities]
const nstates    = length(states_2d)

# Action discretization
const num_points_action = 11
const actions = collect(LinRange(-1, 1, num_points_action))
const nactions = length(actions)

println("Number of states  = $nstates")
println("Number of actions = $nactions")

# Noise standard deviation (pick the first from sigma_list)
const sigma = sigma_list[1]

# Number of random samples used when constructing transitions
const nsamples = 20

##########################################################
# 2) Build the transition probabilities T[s, a, s_next]
##########################################################
# T is a 3D array of size (nstates × nactions × nstates),
# T[s, a, s_next] = Probability of going to s_next from s under action a.

function is_safe(x::Float64, v::Float64)
    # Example "safe" region: x in [-1,1], v in [-1,1]
    return (-1 <= x <= 1) && (-1 <= v <= 1)
end

# Continuous (noisy) dynamics:
#    If in safe region => x_{t+1} = x + v,  v_{t+1} = (v + a) + noise
#    If outside safe   => remain in the same state
function dynamics_rand(x::Float64, v::Float64, u::Float64, sigma::Float64)
    if !is_safe(x, v)
        return (x, v)   # no movement if outside the safe region
    else
        d     = rand(Normal(0, sigma))
        xnext = x + v
        vnext = (v + u) + d
        return (xnext, vnext)
    end
end

# Build a KD-tree for snapping continuous next-states to the nearest discrete state.
states_matrix = hcat([collect(s) for s in states_2d]...)  # shape: 2 x nstates
tree = KDTree(states_matrix)

println("\nBuilding transition probabilities T[s, a, s_next] ...")
T = zeros(Float64, nstates, nactions, nstates)

for s_idx in 1:nstates
    (x, v) = states_2d[s_idx]
    for a_idx in 1:nactions
        u = actions[a_idx]
        # Sample many times and see which next-state we get
        for _ in 1:nsamples
            (xn, vn) = dynamics_rand(x, v, u, sigma)
            # find the nearest discrete neighbor
            idx_neighbors, _ = knn(tree, [xn, vn], 1)
            s_next = idx_neighbors[1]
            T[s_idx, a_idx, s_next] += 1.0
        end
        # Normalize so sum_{s_next} T[s_idx, a_idx, s_next] = 1
        total = sum(T[s_idx, a_idx, :])
        if total > 1e-12
            @inbounds for sn in 1:nstates
                T[s_idx, a_idx, sn] /= total
            end
        end
    end
end
println("Done building T.\n")

##########################################################
# 3) Reward function
##########################################################
# For this example: +1 if in safe region, 0 otherwise
function reward(s::Tuple{Float64, Float64}, a::Float64)
    return is_safe(s[1], s[2]) ? 1.0 : 0.0
end

##########################################################
# 4) Solve the linear program (flow constraints)
##########################################################
function solve_case(
    sigma::Float64,
    center_of_prob::Tuple{Float64,Float64}
)
    println("*************************************************")
    println("Solving case: sigma = $sigma, alpha_center = $center_of_prob")
    println("*************************************************")

    # Setup model
    model = Model(Gurobi.Optimizer)

    # Define z[s, a] and y[s, a]
    @variable(model, z[1:nstates, 1:nactions] >= 0)
    @variable(model, y[1:nstates, 1:nactions] >= 0)

    # c2: flow constraints
    @constraint(model, c2[j in 1:nstates],
        sum(z[s,a] * T[s,a,j] for s in 1:nstates for a in 1:nactions)
        == sum(z[j,a2]       for a2 in 1:nactions)
    )

    # c3: alpha distribution
    # For demonstration, alpha = 1 / nstates (uniform)
    alpha_dist = 1.0 / nstates
    @constraint(model, c3[j in 1:nstates],
        sum(z[j,a] for a in 1:nactions)
      + sum(y[j,a] for a in 1:nactions)
      - sum(y[s,a] * T[s,a,j] for s in 1:nstates, a in 1:nactions)
        == alpha_dist
    )

    # Objective: maximize sum_{s,a} z[s,a] * R(s,a)
    @objective(model, Max, sum(z[s,a]*reward(states_2d[s], actions[a]) 
                               for s in 1:nstates, a in 1:nactions))

    optimize!(model)
    stat = termination_status(model)
    println("Solver status: $stat")
    if stat == MOI.OPTIMAL
        println("Optimal objective value = ", objective_value(model))
    else
        println("No optimal solution found. status = ", stat)
    end

    # Retrieve primal solution: z_sol[s] = sum_{a} z[s,a]
    # Use a numeric vector to store the solution.
    z_sol = zeros(nstates)
    for s in 1:nstates
        z_sol[s] = sum(value(z[s,a]) for a in 1:nactions)
    end

    # Build Nx×Ny matrix for occupation measure z(s).
    Nx = length(positions)
    Ny = length(velocities)
    Z_occup = zeros(Nx, Ny)

    # The indexing in states_2d is row-major: 
    #   states_2d[(i-1)*Ny + j] = (positions[i], velocities[j])
    for i in 1:Nx
        for j in 1:Ny
            sidx = (i - 1)*Ny + j
            Z_occup[i,j] = z_sol[sidx]
        end
    end

    # Prepare meshgrid arrays (X,Y). 
    # X[i,j] = positions[i], Y[i,j] = velocities[j].
    # That means X is Nx×Ny, Y is Nx×Ny
    X = [positions[i]  for j in 1:Ny, i in 1:Nx]  # Nx×Ny
    Y = [velocities[j] for j in 1:Ny, i in 1:Nx]

    # Retrieve duals for c2, c3
    dual_c2_vals = [dual(c2[j]) for j in 1:nstates]
    dual_c3_vals = [dual(c3[j]) for j in 1:nstates]
    G_map = zeros(Nx, Ny)
    H_map = zeros(Nx, Ny)
    for i in 1:Nx
        for j in 1:Ny
            sidx = (i - 1)*Ny + j
            G_map[i,j] = dual_c2_vals[sidx]
            H_map[i,j] = dual_c3_vals[sidx]
        end
    end

    return X, Y, Z_occup, G_map, H_map
end

##########################################################
# 5) Plot in 3D (z, g, h) side-by-side
##########################################################
function main_3D()
    # Solve one test case
    s = sigma_list[1]
    c = center_list[1]
    X, Y, Z_occup, G_map, H_map = solve_case(s, c)

    # "Safe set" boundary lines (for reference)
    safe_x = [-1, 1, 1, -1, -1]
    safe_v = [-1, -1, 1, 1, -1]
    safe_z = zeros(length(safe_x))

    # Create a figure with 3 subplots (each with its own colorbar).
    # We'll do 1 row × 6 columns => (Axis, Colorbar), (Axis, Colorbar), (Axis, Colorbar)
    fig = Figure(resolution=(1800, 600))

    # 1) Occupation measure z(s)
    ax1 = Axis3(fig[1,1],
        title  = "z(s)",
        xlabel = "Position (x)",
        ylabel = "Velocity (v)",
        zlabel = "z(s)"
    )
    plt1 = surface!(ax1, X, Y, Z_occup', colormap=:plasma)
    lines!(ax1, safe_x, safe_v, safe_z, color=:red, linewidth=3)
    Colorbar(fig[1,2], plt1, label="z(s)")

    # 2) Dual w.r.t. flow constraint: G_map
    ax2 = Axis3(fig[1,3],
        title  = "Dual g(s)",
        xlabel = "Position (x)",
        ylabel = "Velocity (v)",
        zlabel = "g(s)"
    )
    plt2 = surface!(ax2, X, Y, G_map', colormap=:viridis)
    lines!(ax2, safe_x, safe_v, safe_z, color=:red, linewidth=3)
    Colorbar(fig[1,4], plt2, label="g(s)")

    # 3) Dual w.r.t. alpha-dist constraint: H_map
    ax3 = Axis3(fig[1,5],
        title  = "Dual h(s)",
        xlabel = "Position (x)",
        ylabel = "Velocity (v)",
        zlabel = "h(s)"
    )
    plt3 = surface!(ax3, X, Y, H_map', colormap=:hot)
    lines!(ax3, safe_x, safe_v, safe_z, color=:red, linewidth=3)
    Colorbar(fig[1,6], plt3, label="h(s)")

    display(fig)
    println("\nAll subplots shown in one row with separate colorbars.")
    return fig
end

##########################################################
# RUN
##########################################################
main_3D()
