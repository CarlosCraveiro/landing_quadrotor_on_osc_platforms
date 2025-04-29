function verify_landing(x, x_ref)
    #crit = [0.05; 0.05; 0.05; 0.10; 0.02; 0.1]
    crit = [0.05; 0.05; 0.05; 0.05; 0.05; 0.05]

    sucess_check = true
    for i = 1:(Nx)
        if (abs(x[i] - x_ref[i]) >= crit[i])
             sucess_check = false
        end
    end
    return ~sucess_check
end

function closed_loop(x0, state_space, tunning_params, traj_params, simulation, model, controller)
    local h_c = simulation.h_controller
    local h_uni = simulation.h_universe

    local N_c = simulation.Nt_controller
    local N_uni = simulation.Nt_universe

    local umax = model.umax
    local umin = model.umin

    xhist = zeros(length(x0), N_uni)

    u0 = controller(1, x0)

    uhist = zeros(length(u0), N_uni - 1)
    plathist = zeros(N_uni)
    drefyhist = zeros(N_uni)
    refyhist = zeros(N_uni)

    uhist[:,1] .= u0
    xhist[:,1] .= x0

    local not_landed = true
    land_time = 0.0

    uk = u0
    N_ratio = Int(round(h_c / h_uni)) ## h_c need to be multiple of h_u

    for k = 1:(N_uni-1)
        local curr_time_inst = k * h_uni

        if(k % N_ratio == 0)
            uk = controller(curr_time_inst, xhist[:, k])
        end

        if not_landed
            not_landed = verify_landing(xhist[:, k], gen_ref(state_space, traj_params, tunning_params, curr_time_inst, h_uni))
            land_time = curr_time_inst
        end

        foo = gen_ref(state_space, traj_params, tunning_params, curr_time_inst, h_uni)
        refyhist[k] = foo[2]
        drefyhist[k] = foo[5]

        plathist[k] = traj_y(traj_params, curr_time_inst) - traj_params.stat_height # Recover the current platform height
        if k == N_uni # Goes to the N_uni - 1 step
            continue
        end

        uhist[:, k] = max.(min.(umax, uk), umin) #enforce control limits

        local height = xhist[2, k] - plathist[k]
        xhist[:, k + 1] .= dynamics_rk4(xhist[:, k],uhist[:, k],(x,u)->quad_dynamics(model, x, u, height), simulation)
    end

    return xhist, uhist, plathist, refyhist, drefyhist, land_time
end