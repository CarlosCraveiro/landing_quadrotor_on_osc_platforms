"""
Todo..
"""
struct Simulation
    h_universe::Float64      # Universe Time Step
    h_controller::Float64    # Controler Time Step
    Tfinal::Float64          # Final simulation time (it always start from time 0)
    Nt_universe::Int64       # Universe Number of time-steps
    Nt_controller::Int64     # Controller Number of time-steps

    function Simulation(h_universe, h_controller, Tfinal)
        Nt_universe = Int(Tfinal/h_universe) + 1    # universe number of time steps
        Nt_controller = Int(Tfinal/h_controller) + 1    # controller number of time steps

        new(h_universe, h_controller, Tfinal, Nt_universe, Nt_controller)
    end
end

"""
    TODO...
"""
function dynamics_rk4(x,u, dynamics, sim_param)
    # RK4 integration with zero-order hold on u
    local h_u = sim_param.h_universe

    # VERIFICAR A ALTURA ADEQUADA DA PLATAFORMA (Se é adequada para o RK4)
    f1 = dynamics(x, u)
    f2 = dynamics(x + 0.5 * h_u * f1, u)
    f3 = dynamics(x + 0.5 * h_u * f2, u)
    f4 = dynamics(x + h_u * f3, u)
    xn = x + (h_u / 6.0)*(f1 + 2*f2 + 2*f3 + f4)
    
    xn[4:7] .= xn[4:7]/norm(xn[4:7]) #re-normalize quaternion
    
    return xn
end