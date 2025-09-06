"""
    lqr_controller(model::Model, simulation::Simulation, tunning_params::MPCTunningParameters, traj_params::Trajectory, lstate_space::LinearStateSpace, t::Float64, x::Vector{Float64}, xref::Vector{Float64})

Computes the control input using a Linear Quadratic Regulator (LQR) based on the current state and a reference state.

This function calculates the control input required to drive the system's state `x` towards a reference state `xref` using the LQR control law. It linearizes the attitude error using Rodriguez parameters.

# Arguments
- `model::Model`: The `Model` struct containing the quadrotor's physical parameters.
- `simulation::Simulation`: The `Simulation` struct containing the simulation parameters.
- `tunning_params::MPCTunningParameters`: The `MPCTunningParameters` struct containing the LQR tuning matrices `Q` and `R`.
- `traj_params::Trajectory`: The `Trajectory` struct defining the desired motion (used here for the reference).
- `lstate_space::LinearStateSpace`: The `LinearStateSpace` struct containing the discrete-time linearized state-space matrices `A_r` and `B_r`, and the hover control input `u_hover`.
- `t`: The current simulation time [s].
- `x::Vector{Float64}`: The current state vector of the quadrotor.
- `xref::Vector{Float64}`: The reference state vector at the current time (its a matrix to be compatible with mpc_controller call).

# Returns
- `Vector{Float64}`: The computed control input vector.
"""
function lqr_controller(
    model::Model,
    simulation::Simulation,
    tunning_params::MPCTunningParameters,
    traj_params::Trajectory,
    lstate_space::LinearStateSpace, # linear state space
    t,
    x::Vector{Float64},
    xref::Vector{Float64}
    )

    K_matrix = dlqr(lstate_space.A_r, lstate_space.B_r, tunning_params.Q, tunning_params.R)
    ϕ = qtorp(L(xref[4:7])'*x[4:7])
    Δx̃ = [x[1:3] - xref[1:3]; ϕ; x[8:10] - xref[8:10]; x[11:13] - xref[11:13]]
    
    return lstate_space.u_hover - K_matrix * Δx̃ 
end