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
function traj_z(traj_params, t)
    local A = traj_params.Amp
    local f = traj_params.freq
    local offset = traj_params.stat_height

    return A * sin(2*pi*f*t) + offset
end

"""
    TODO...
"""
function traj_z_dot(traj_params, t)
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
    # x_ref needs to be full length size - LINEAR SIZE + 1
    x_ref = zeros((Nx + 1) * Nh) # Initialize the reference with zeros

    for j = 1:Nh # for each element on the predictive horizon
        x_ref[(Nx + 1)*(j-1) + 3] = traj_z(traj_params, t+(j*h_c))
        x_ref[(Nx + 1)*(j-1) + 4] = 1.0
        x_ref[(Nx + 1)*(j-1) + 10] = traj_z_dot(traj_params, (t+(j*h_c)))
    end

    return x_ref
end