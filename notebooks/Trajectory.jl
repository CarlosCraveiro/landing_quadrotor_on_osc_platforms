"""
    Defines the trajectory parameters, the trajectory for now is a sine
"""
mutable struct Trajectory
    Amp::Float64    # Sine amplitude - [meters]
    freq::Float64   # Sine frequency - [Hz]
    init_heigth::Float64 # Initial height [m]
    stat_height::Float64 # Stationary height [m]
end

"""
    TODO...
"""
function traj_y(traj_params, t)
    local A = traj_params.Amp
    local f = traj_params.freq
    local offset = traj_params.stat_height

    return A * sin(2*pi*f*t) + offset
end
"""
    TODO...
"""
function traj_y_dot(traj_params, t)
    local A = traj_params.Amp
    local f = traj_params.freq

    return (A*2*pi*f) * cos(2*pi*f*t)
end

"""
    h_c - time step (controller)
"""
function gen_ref(state_space, traj_params, tunning_params, t, h_c)
    local Nh = tunning_params.Nh
    local Nx = state_space.Nx
    x_ref = zeros(Nx * Nh) # Initialize the reference with zeros

    for j = 1:Nh # for each element on the predictive horizon
        x_ref[Nx*(j-1) + 2] = traj_y(traj_params, t+(j*h_c))
        x_ref[Nx*(j-1) + 5] = traj_y_dot(traj_params, (t+(j*h_c)))
    end

    return x_ref
end