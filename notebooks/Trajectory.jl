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
function gen_ref(model, traj_params, tunning_params, t, h_c)
    z_pos = 3 # Position of z on the state vector
    dz_pos = 10  # Position of dz on the state vect
    Nh = tunning_params.Nh
    Nx_full = model.Nx
    
    # x_ref needs to be full length size - LINEAR SIZE + 1
    x_ref = zeros(Nx_full * Nh) # Initialize the reference with zeros

    for j = 1:Nh # for each element on the predictive horizon
        x_ref[Nx_full*(j-1) + z_pos] = traj_z(traj_params, t+(j*h_c))
        x_ref[Nx_full*(j-1) + 4] = 1.0 # Asserts the reference quaternion to be q = [1 0 0 0] 
        x_ref[Nx_full*(j-1) + dz_pos] = traj_z_dot(traj_params, (t+(j*h_c)))
    end

    return x_ref
end