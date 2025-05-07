function lqr_controller(
        model, 
        simulation,
        tunning_params,
        traj_params,
        lstate_space, # linear state space
        t,
        x,
        xref
    )

    K_matrix = dlqr(lstate_space.A_r, lstate_space.B_r, tunning_params.Q, tunning_params.R)
    ϕ = qtorp(L(xref[4:7])'*x[4:7])
    Δx̃ = [x[1:3] - xref[1:3]; ϕ; x[8:10] - xref[8:10]; x[11:13] - xref[11:13]]
    
    return lstate_space.u_hover - K_matrix * Δx̃ 
end