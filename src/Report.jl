using Plots

function plot_state(index::Int, label::String, unit::String, out, out_lqr, thist; max_points::Int = 1000)
    # Compute stride for downsampling
    stride = max(1, floor(Int, length(thist) / max_points))

    # Apply stride to time
    thist_sampled = thist[1:stride:end]

    # Sample MPC and LQR data for this state
    state_mpc_sampled = out.x_quad[index, 1:stride:end]
    state_lqr_sampled = out_lqr.x_quad[index, 1:stride:end]
    state_ref_sampled = out.x_ref[index, 1:stride:end]
    state_plat_sampled = out.x_plat[index, 1:stride:end]

    # --- Create plot ---
    plt = plot(thist_sampled, state_mpc_sampled, label = "$label - MPC", lw = 2)
    plot!(plt, thist_sampled, state_lqr_sampled, label = "$label - LQR", lw = 2)
    plot!(plt, thist_sampled, state_ref_sampled, label = "Trajetória de referência", lw = 2, ls = :dash, color = :black)
    plot!(plt, thist_sampled, state_plat_sampled, label = "Plataforma", lw = 2, color = :green)

    # Scatter points for landing marks
    scatter!([out.land_time], [out.x_quad[index, out.land_mark]],
         label = "Pouso MPC", color = :red, markersize = 4)
    scatter!([out_lqr.land_time], [out_lqr.x_quad[index, out_lqr.land_mark]],
        label = "Pouso LQR", color = :purple, markersize = 4)

    # Labels and title
    xlabel!(plt, "Tempo (s)")
    ylabel!(plt, "Estado $label [$unit]")
    title!(plt, "Evolução do estado $label (MPC vs LQR)")
    savefig("plots/estado_$label.svg")
    
    return plt
end

function plot_attitude_error(out, out_lqr, thist; max_points::Int=1000, show_ref_line::Bool=true, 
                                title_str::AbstractString="Erro de atitude — qtorp(L(q_ref)' * q)")
    # =================== Parâmetros de amostragem ===================
    max_points = 1000
    stride = max(1, floor(Int, length(thist) / max_points))
    t = thist[1:stride:end]

    # =================== Utils locais ===================
    normalize_quat(q) = q ./ sqrt(sum(abs2, q))
    align_sign!(q, qref) = (dot(qref, q) < 0) ? (-q) : q  # retorna q alinhado ao qref

    # =================== Buffers de erro ===================
    n = length(t)
    e_mpc = zeros(3, n)
    e_lqr = zeros(3, n)

    # =================== Cálculo do erro amostrado ===================
    nq = size(out.x_quad, 2)
    @assert size(out.x_ref, 2) >= nq "x_ref deve cobrir o horizonte de x_quad"
    @assert size(out_lqr.x_quad, 2) >= nq "x_quad do LQR com tamanho compatível"

    k_s = 0
    for k in 1:stride:nq
        k_s += 1

        # Referência e estados (unitários + alinhados)
        q_ref = normalize_quat(@view out.x_ref[4:7, k])

        q_m  = normalize_quat(@view out.x_quad[4:7, k])
        q_m  = align_sign!(q_m, q_ref)

        q_l  = normalize_quat(@view out_lqr.x_quad[4:7, k])
        q_l  = align_sign!(q_l, q_ref)

        # Erro de atitude via qtorp(L(q_ref)' * q)
        q_err_m = L(q_ref)' * q_m
        q_err_l = L(q_ref)' * q_l

        e_mpc[:, k_s] .= qtorp(q_err_m)
        e_lqr[:, k_s] .= qtorp(q_err_l)
    end

    # =================== Valores no instante de pouso ===================
    # MPC
    kM = out.land_mark
    q_ref_M = normalize_quat(out.x_ref[4:7, kM])
    q_m_M   = normalize_quat(out.x_quad[4:7, kM])
    q_m_M   = align_sign!(q_m_M, q_ref_M)
    e_mpc_land = qtorp(L(q_ref_M)' * q_m_M)

    # LQR (usa a mesma referência time-varying do 'out')
    kL = out_lqr.land_mark
    q_ref_L = normalize_quat(out.x_ref[4:7, kL])
    q_l_L   = normalize_quat(out_lqr.x_quad[4:7, kL])
    q_l_L   = align_sign!(q_l_L, q_ref_L)
    e_lqr_land = qtorp(L(q_ref_L)' * q_l_L)

    # =================== Plot ===================
    plt = plot()
    comp_labels = ["e₁", "e₂", "e₃"]

    # Curvas MPC (linha contínua)
    for i in 1:3
        plot!(plt, t, e_mpc[i, :], lw=2, label = "$(comp_labels[i]) MPC")
    end

    # Curvas LQR (linha tracejada)
    for i in 1:3
        plot!(plt, t, e_lqr[i, :], lw=2, ls=:dash, label = "$(comp_labels[i]) LQR")
    end

    # Linha de referência (zero)
    hline!(plt, [0.0], ls=:dot, label="Referência")

    # Marcadores de pouso
    for i in 1:3
        scatter!(plt, [out.land_time],     [e_mpc_land[i]], label = "Pouso MPC ($(comp_labels[i]))",
            color=:red, markersize=4)
        scatter!(plt, [out_lqr.land_time], [e_lqr_land[i]], label = "Pouso LQR ($(comp_labels[i]))",
            color=:purple, markersize=4)
    end

    xlabel!(plt, "Tempo (s)")
    ylabel!(plt, "Erro de atitude (vetor de Gibbs)")
    title!(plt, "Erro de atitude — qtorp(L(q_ref)' * q): MPC vs LQR")
    savefig("plots/atitude.svg")
    return plt
end

function plot_u(index::Int, out, out_lqr, thist, model, max_points::Int = 1000)
    # Compute stride for downsampling
    stride = max(1, floor(Int, length(thist) / max_points))

    # Apply stride to time
    thist_sampled = thist[1:stride:end]
    
    # Extract the sampled control signals
    u_mpc_sampled = out.u_quad[index, 1:stride:end]
    u_lqr_sampled = out_lqr.u_quad[index, 1:stride:end]

    # Match vector sizes
    n = minimum((length(u_mpc_sampled), length(u_lqr_sampled), length(thist_sampled)))
    t = thist_sampled[1:n]
    u_mpc = u_mpc_sampled[1:n]
    u_lqr = u_lqr_sampled[1:n]

    # Get limits
    umin = model.umin[index]
    umax = model.umax[index]

    # Plot
    plt = plot(t, u_mpc, label = "u$(index) MPC", lw = 2)
    plot!(plt, t, u_lqr, label = "u$(index) LQR", lw = 2)

    # Add horizontal lines for limits
    hline!(plt, [umin], label = "u$(index) min", ls = :dash)
    hline!(plt, [umax], label = "u$(index) max", ls = :dash)

    xlabel!(plt, "Tempo")
    ylabel!(plt, "Comando de controle u$(index)")
    title!(plt, "Entrada de controle $(index) (MPC vs LQR) + Limites")

    savefig("plots/control_input_$(index).svg")
    return plt
end