"""
    Trajectory

Defines the parameters for a sinusoidal trajectory in the z-direction.

# Fields
- `Amp::Float64`: Amplitude of the sinusoidal trajectory [meters].
- `freq::Float64`: Frequency of the sinusoidal trajectory [Hz].
- `init_heigth::Float64`: Initial height [m] (though not directly used in the current trajectory functions).
- `stat_height::Float64`: Stationary or offset height of the sinusoidal trajectory [m].
"""
mutable struct Trajectory
    Amp::Float64    # Sine amplitude - [meters]
    freq::Float64   # Sine frequency - [Hz]
    init_heigth::Float64 # Initial height [m]
    stat_height::Float64 # Stationary height [m]
end

"""
    traj_z(traj_params::Trajectory, t::Float64)

Calculates the desired z-position at a given time based on the trajectory parameters.

The trajectory is defined as a sinusoidal function of time with a specified amplitude and frequency, offset by a stationary height.

# Arguments
- `traj_params::Trajectory`: The `Trajectory` struct containing the trajectory parameters.
- `t::Float64`: The current time [s].

# Returns
- `Float64`: The desired z-position at the given time [m].
"""
function traj_z(traj_params::Trajectory, t::Float64)
    local A = traj_params.Amp
    local f = traj_params.freq
    local offset = traj_params.stat_height

    return A * sin(2*pi*f*t) + offset
end

"""
    traj_z_dot(traj_params::Trajectory, t::Float64)

Calculates the desired z-velocity at a given time based on the trajectory parameters.

This function computes the first derivative of the `traj_z` function with respect to time, yielding the desired vertical velocity.

# Arguments
- `traj_params::Trajectory`: The `Trajectory` struct containing the trajectory parameters.
- `t::Float64`: The current time [s].

# Returns
- `Float64`: The desired z-velocity at the given time [m/s].
"""
function traj_z_dot(traj_params::Trajectory, t::Float64)
    local A = traj_params.Amp
    local f = traj_params.freq

    return (A*2*pi*f) * cos(2*pi*f*t)
end


"""
    gen_ref(model::Model, traj_params::Trajectory, tunning_params::NamedTuple, t, h_c::Float64)

Generates a reference trajectory for the predictive horizon, primarily for the z-position and its velocity.

This function computes the desired state vector over the prediction horizon `Nh`, based on the trajectory parameters and the current time. For now, it focuses on generating references for the z-position and sets the reference quaternion to represent no rotation.

# Arguments
- `model::Model`: The `Model` struct containing the system's dimensions, including the full state vector size `Nx_full`.
- `traj_params::Trajectory`: The `Trajectory` struct defining the desired motion.
- `tunning_params::MPCTunningParameters A named tuple containing tuning parameters for the controller, including the prediction horizon `Nh`.
- `t`: The current time [s].
- `h_c::Float64`: The control time step [s].

# Returns
- `Vector{Float64}`: A vector containing the reference state over the prediction horizon. Each segment of the vector corresponds to a future time step in the horizon and contains the full state reference. Currently, only the z-position and z-velocity are actively generated from the trajectory, with other states (like x, y, roll, pitch, yaw rates) being zero and the quaternion set to `[1, 0, 0, 0]`.
"""
function gen_ref(
        model::Model,
        traj_params::Trajectory,
        tunning_params::MPCTunningParameters,
        t,
        h_c::Float64
    )
    z_pos = 3 # Position of z on the state vector
    dz_pos = 10  # Position of dz on the state vect
    Nh = tunning_params.Nh
    Nx_full = model.Nx
    1
    x_ref = zeros(Nx_full * Nh) # Initialize the reference with zeros

    for j = 1:Nh # for each element on the predictive horizon
        x_ref[Nx_full*(j-1) + z_pos] = traj_z(traj_params, t+(j*h_c))
        x_ref[Nx_full*(j-1) + 4] = 1.0 # Asserts the reference quaternion to be q = [1 0 0 0] 
        x_ref[Nx_full*(j-1) + dz_pos] = traj_z_dot(traj_params, (t+(j*h_c)))
    end

    return x_ref
end