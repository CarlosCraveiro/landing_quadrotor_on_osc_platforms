include("Model_nd_Dynamics.jl")

"""
    verify_landing(x::Vector{Float64}, xref::Vector{Float64}, z_idx::Int64, dz_idx::Int64) -> Bool

Determines if the quadrotor has landed by checking if its vertical position (`z`) 
and vertical velocity (`dz`) are within 0.05 of the reference values.

# Arguments
- `x`: Current full state vector of the quadrotor.
- `xref`: Desired reference state vector.
- `z_idx`: Index of the vertical position in the state vector.
- `dz_idx`: Index of the vertical velocity in the state vector.

# Returns
- `Bool`: `true` if both `z` and `dz` are within the threshold, `false` otherwise.
"""
function verify_landing(x::Vector{Float64}, xref::Vector{Float64}, z_idx::Int64, dz_idx::Int64)
    # Create new vectors using only z and dz components
    x_sub    = [x[z_idx], x[dz_idx]]
    xref_sub = [xref[z_idx], xref[dz_idx]]
    
    return maximum(abs, x_sub - xref_sub) < 0.05
end

"""
    SimResults

A mutable struct to store the results of the closed-loop simulation.

# Fields
- `x_quad::Matrix{Float64}`: Matrix storing the full state vector of the quadrotor at each time step (state_length x N_points).
- `u_quad::Matrix{Float64}`: Matrix storing the control input vector applied to the quadrotor at each time step (control_length x (N_points - 1)).
- `x_plat::Matrix{Float64}`: Matrix storing the state vector of the platform (typically the desired landing target) at each time step (state_length x N_points).
- `x_ref::Matrix{Float64}`: Matrix storing the reference state vector for the quadrotor at each time step (state_length x N_points).
- `land_time::Float64`: The time at which landing was detected.
- `not_landed::Bool`: A flag indicating whether the quadrotor has landed (`false`) or is still in flight (`true`).
- `N_points::Int32`: The total number of time steps in the simulation.
- `land_mark::Int32`: The time step index at which landing was detected.

# Constructors
- `SimResults(state_length::Int, control_length::Int, N_points::Int)`: Constructs a `SimResults` object, initializing the data storage matrices with zeros and setting the initial landing status.
"""
mutable struct SimResults
    x_quad::Matrix{Float64}
    u_quad::Matrix{Float64}
    x_plat::Matrix{Float64}
    x_ref::Matrix{Float64}
    land_time::Float64
    not_landed::Bool
    N_points::Int32
    land_mark::Int32

    function SimResults(state_length, control_length, N_points)
        x_quad = zeros(state_length, N_points)
        x_plat = zeros(state_length, N_points)
        x_ref = zeros(state_length, N_points)
        u_quad = zeros(control_length, N_points - 1)
        land_time = 0
        not_landed = true
        land_mark = 0
        new(x_quad, u_quad, x_plat, x_ref, land_time, not_landed, N_points, land_mark)
    end
end


"""
    closed_loop(x0::Vector{Float64}, state_space::LinearStateSpace, tunning_params::MPCTunningParameters,
                traj_params::Trajectory, simulation::Simulation, model::Model, controller::Function,
                wind_vector::Vector{Float64}, wind_vel::Float64) -> SimResults

Runs a closed-loop simulation of the quadrotor and platform system for the full simulation
duration, updating states using the given controller and dynamics at each universe time step.

# Arguments
- `x0`: Initial full state vector of the quadrotor.
- `state_space`: Linearized system dynamics.
- `tunning_params`: MPC tuning parameters.
- `traj_params`: Desired trajectory parameters.
- `simulation`: Simulation configuration (time steps, durations).
- `model`: Quadrotor model parameters.
- `controller`: Control function computing the control input.
- `wind_vector`: Direction of external wind disturbance.
- `wind_vel`: Magnitude of external wind disturbance.

# Returns
- `SimResults`: Struct containing the histories of states, control inputs, reference trajectories,
  platform states, landing time, and landing status.
"""
function closed_loop(
        x0::Vector{Float64},
        state_space::LinearStateSpace,
        tunning_params::MPCTunningParameters,
        traj_params::Trajectory,
        simulation::Simulation,
        model::Model,
        controller::Function,
        wind_vector::Vector{Float64},
        wind_vel::Float64
    )
    Nx_full = model.Nx
    z_pos = 3 # Position of z on the state vector
    dz_pos = 10  # Position of dz on the state vector
    
    h_c = simulation.h_controller
    h_uni = simulation.h_universe
    N_c = simulation.Nt_controller
    N_uni = simulation.Nt_universe
    umax = model.umax
    umin = model.umin
    
    out = SimResults(model.Nx, model.Nu, N_uni)
        
    # CONTROLER INPUT FULL SIZE VECTOR
    u0 = controller(1, x0)

    out.u_quad[:,1] .= u0
    out.x_quad[:,1] .= x0

    uk = u0
    N_ratio = Int(round(h_c / h_uni)) ## h_c need to be multiple of h_u

    for k = 1:(N_uni-1)
        curr_time_inst = k * h_uni

        if(k % N_ratio == 0)
            uk = controller(curr_time_inst, out.x_quad[:, k])
        end

        if out.not_landed
            out.not_landed = ~verify_landing(out.x_quad[:, k], gen_ref(model, traj_params, tunning_params, curr_time_inst, h_uni), z_pos, dz_pos)
            out.land_time = curr_time_inst
            out.land_mark = k
        end

        out.x_ref[Nx_full*(k-1) + 1:Nx_full*(k)] = gen_ref(model, traj_params, tunning_params, curr_time_inst, h_uni)[1:Nx_full]
        
        out.x_plat[Nx_full*(k-1) + 1:Nx_full*(k)] = out.x_ref[Nx_full*(k-1) + 1:Nx_full*(k)]
        out.x_plat[Nx_full*(k-1) + z_pos] = out.x_plat[Nx_full*(k-1) + z_pos] - traj_params.stat_height

        out.u_quad[:, k] = max.(min.(umax, uk), umin) # enforce control limits

        height = out.x_quad[z_pos, k] - out.x_plat[z_pos, k]

        # DYNAMICS RK4
        out.x_quad[:, k + 1] .= dynamics_rk4(out.x_quad[:, k], out.u_quad[:, k],(x,u)->quad_dynamics(model, x, u, height, wind_vector, wind_vel), simulation.h_universe)
        
        if k == (N_uni - 1)
            out.x_ref[Nx_full*k + 1:Nx_full*(k + 1)] = gen_ref(model, traj_params, tunning_params, (k + 1) * h_uni, h_uni)[1:Nx_full]
            out.x_plat[Nx_full*k + 1:Nx_full*(k + 1)] = out.x_ref[Nx_full*k + 1:Nx_full*(k + 1)]
            out.x_plat[Nx_full*k + z_pos] = out.x_plat[Nx_full*k + z_pos] - traj_params.stat_height
        end
    end

    return out
end