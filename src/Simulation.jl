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
    dynamics_rk4(x::Vector{Float64}, u::Vector{Float64}, dynamics::Function, sim_param::Simulation)

Integrates the system dynamics one time step using the fourth-order Runge-Kutta (RK4) method with zero-order hold on the control input.

This function takes the current state, control input, the dynamics function, and simulation parameters to compute the next state of the system. It also ensures that the quaternion part of the state vector remains normalized.

# Arguments
- `x`: The current state vector of the system.
- `u`: The control input vector, held constant over the integration step.
- `dynamics::Function`: A function that computes the time derivative of the state vector, i.e., `dx = dynamics(x, u)`. This function should accept the state and control input as arguments and return the state derivative.
- `sim_param::Simulation`: The `Simulation` struct containing the simulation parameters, including the universe time step `h_universe`.

# Returns
- `Vector{Float64}`: The state vector at the next time step, after one RK4 integration.
"""
function dynamics_rk4(
        x, 
        u, 
        dynamics::Function, 
        h_step
    )
    # RK4 integration with zero-order hold on u
    #local h_u = sim_param.h_universe

    f1 = dynamics(x, u)
    f2 = dynamics(x + 0.5 * h_step * f1, u)
    f3 = dynamics(x + 0.5 * h_step * f2, u)
    f4 = dynamics(x + h_step * f3, u)
    xn = x + (h_step / 6.0)*(f1 + 2*f2 + 2*f3 + f4)
    
    xn[4:7] .= xn[4:7]/norm(xn[4:7]) #re-normalize quaternion
    
    return xn
end