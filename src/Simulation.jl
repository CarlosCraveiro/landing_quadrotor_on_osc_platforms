"""
    Simulation

A struct that holds the parameters for the simulation environment.

# Fields
- `h_universe::Float64`: Time step for the universe dynamics (s).
- `h_controller::Float64`: Time step for the controller updates (s).
- `Tfinal::Float64`: Total simulation time (s), starting from time 0.
- `Nt_universe::Int64`: Total number of time steps in the universe simulation.
- `Nt_controller::Int64`: Total number of time steps for the controller.

# Constructors
- `Simulation(h_universe::Float64, h_controller::Float64, Tfinal::Float64)`: Constructs a `Simulation` object, calculating the number of time steps for both the universe and the controller based on the provided time steps and final time.
"""
struct Simulation
    h_universe::Float64      # Universe Time Step
    h_controller::Float64    # Controler Time Step
    Tfinal::Float64          # Final simulation time (it always start from time 0)
    Nt_universe::Int64       # Universe Number of time-steps
    Nt_controller::Int64     # Controller Number of time-steps

    function Simulation(h_universe::Float64, h_controller::Float64, Tfinal::Float64)
        Nt_universe = Int(Tfinal/h_universe) + 1    # universe number of time steps
        Nt_controller = Int(Tfinal/h_controller) + 1    # controller number of time steps

        new(h_universe, h_controller, Tfinal, Nt_universe, Nt_controller)
    end
end

"""
    dynamics_rk4(x::Vector{Float64}, u::Vector{Float64}, dynamics::Function, h_step::Float64) -> Vector{Float64}

Integrates the system dynamics over one time step using the fourth-order Runge-Kutta (RK4) method
with zero-order hold on the control input. The quaternion part of the state vector is re-normalized
after integration.

# Arguments
- `x`: Current state vector.
- `u`: Control input vector, assumed constant during the step.
- `dynamics`: Function that computes the state derivative, `dx = dynamics(x, u)`.
- `h_step`: Integration step size.

# Returns
- `Vector{Float64}`: State vector at the next time step.
"""
function dynamics_rk4(
        x, 
        u, 
        dynamics::Function, 
        h_step
    )
    # RK4 integration with zero-order hold on u
    f1 = dynamics(x, u)
    f2 = dynamics(x + 0.5 * h_step * f1, u)
    f3 = dynamics(x + 0.5 * h_step * f2, u)
    f4 = dynamics(x + h_step * f3, u)
    xn = x + (h_step / 6.0)*(f1 + 2*f2 + 2*f3 + f4)
    
    xn[4:7] .= xn[4:7]/norm(xn[4:7]) #re-normalize quaternion
    
    return xn
end