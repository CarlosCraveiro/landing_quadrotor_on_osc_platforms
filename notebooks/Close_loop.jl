include("Model_nd_Dynamics.jl")

"""
    TODO...
"""
function verify_landing(x, xref)
    return infinity_norm_of_difference(x, xref) < 0.05
end

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
    x0 - Initial state - Full Vector (13 size)
"""
function closed_loop(x0, state_space, tunning_params, traj_params, simulation, model, controller)
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
            # CONTROLLER xhist -> FULL SIZE STATE VECTOR LENGTH
            uk = controller(curr_time_inst, out.x_quad[:, k])
        end

        if out.not_landed
            # VERIFY LANDING -- LENGTH -> FULL SIZE VECTOR, XHIST
            out.not_landed = ~verify_landing(out.x_quad[:, k], gen_ref(state_space, traj_params, tunning_params, curr_time_inst, h_uni))
            out.land_time = curr_time_inst
            out.land_mark = k
        end

        out.x_ref[Nx_full*(k-1) + 1:Nx_full*(k)] = gen_ref(state_space, traj_params, tunning_params, curr_time_inst, h_uni)[1:Nx_full]
        
        out.x_plat[Nx_full*(k-1) + 1:Nx_full*(k)] = out.x_ref[Nx_full*(k-1) + 1:Nx_full*(k)]
        out.x_plat[Nx_full*(k-1) + z_pos] = out.x_plat[Nx_full*(k-1) + z_pos] - traj_params.stat_height

        out.u_quad[:, k] = max.(min.(umax, uk), umin) #enforce control limits

        height = out.x_quad[z_pos, k] - out.x_plat[z_pos, k]
        
        # DYNAMICS_RK4 - FULL STATE VECTOR LENGTH
        out.x_quad[:, k + 1] .= dynamics_rk4(out.x_quad[:, k], out.u_quad[:, k],(x,u)->quad_dynamics(model, x, u, height), simulation)
        
        if k == (N_uni - 1)
            out.x_ref[Nx_full*k + 1:Nx_full*(k + 1)] = gen_ref(state_space, traj_params, tunning_params, (k + 1) * h_uni, h_uni)[1:Nx_full]
            out.x_plat[Nx_full*k + 1:Nx_full*(k + 1)] = out.x_ref[Nx_full*k + 1:Nx_full*(k + 1)]
            out.x_plat[Nx_full*k + z_pos] = out.x_plat[Nx_full*k + z_pos] - traj_params.stat_height
        end
    end

    return out
end