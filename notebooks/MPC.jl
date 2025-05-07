import Pkg
Pkg.activate(@__DIR__)

using LinearAlgebra
using SparseArrays
using ForwardDiff
using ControlSystems
using OSQP

include("Quaternion.jl")

"""
    MPCTunningParameters

A mutable struct to hold the tuning parameters for the Model Predictive Controller (MPC).

# Fields
- `Nh::Int64`: Predictive horizon - the number of future time steps the controller considers.
- `Nc::Int64`: Control horizon - the number of future time steps over which the control inputs are optimized (Nc <= Nh).
- `Q::Matrix{Float64}`: Stage State Cost Penalty - a positive semi-definite matrix penalizing deviations from the desired state at each step within the prediction horizon.
- `R::Matrix{Float64}`: Stage Control Cost Penalty - a positive definite matrix penalizing the control effort at each step within the control horizon.
- `Qn::Matrix{Float64}`: Terminal State Cost Penalty - a positive semi-definite matrix penalizing the deviation from the desired state at the end of the prediction horizon.
"""
mutable struct MPCTunningParameters
    Nh::Int64           # Predictive horizon
    Nc::Int64           # Control horizon (Nc < Nh)
    Q::Matrix{Float64}  # Stage State Cost Penalty
    R::Matrix{Float64}  # Stage Control Cost Penalty
    Qn::Matrix{Float64} # Terminal State Cost Penalty
end

"""
    LinearStateSpace

A struct that holds the discrete-time linear state-space representation of the quadrotor dynamics around a hover point.

# Fields
- `Nx::Int64`: Dimension of the full state vector.
- `Nu::Int64`: Dimension of the control input vector.
- `A::Matrix{Float64}`: Discrete-time state matrix for the full state.
- `B::Matrix{Float64}`: Discrete-time control matrix for the full state.
- `A_r::Matrix{Float64}`: Discrete-time state matrix for the reduced (controllable) state.
- `B_r::Matrix{Float64}`: Discrete-time control matrix for the reduced (controllable) state.
- `x_hover::Vector{Float64}`: Full state vector at the linearization point (hover).
- `u_hover::Vector{Float64}`: Control input vector at the linearization point (hover thrust).

# Constructors
- `LinearStateSpace(model::Model, simulation::Simulation, Nx::Int64, Nu::Int64, height::Float64)`: Constructs a `LinearStateSpace` object by linearizing the nonlinear quadrotor dynamics around the hover conditions at the specified height. It uses ForwardDiff to compute the Jacobians of the discrete-time dynamics with respect to the state and control input. The state-space matrices are also reduced to ensure controllability.
"""
struct LinearStateSpace
    Nx::Int64          # State vector dimension
    Nu::Int64          # Control vector dimension
    A::Matrix{Float64} # State Matrix
    B::Matrix{Float64} # Control Matrix
    A_r::Matrix{Float64} # Reduced State Matrix
    B_r::Matrix{Float64} # Reduced Control Matrix
    x_hover::Vector{Float64}   # x value for which the linearization was made (13 size, full)
    u_hover::Vector{Float64}   # u value for which the linearization was made

    function LinearStateSpace(model, simulation, Nx, Nu, height)
        x_hover, u_hover = find_hover_conditions(model, height)
        A = ForwardDiff.jacobian(x->
            dynamics_rk4(x, u_hover, (x,u)->quad_dynamics(model, x, u, height), simulation)
            ,x_hover)
        B = ForwardDiff.jacobian(u ->
            dynamics_rk4(x_hover, u, (x,u)->quad_dynamics(model, x, u, height), simulation)
        , u_hover)
        
        # Reduced system - Ensures controlability
        q_hover = x_hover[4:7]
        A_r = Array(E(q_hover)' * A * E(q_hover))
        B_r = Array(E(q_hover)' * B);
        
        new(Nx, Nu, A, B, A_r, B_r, x_hover, u_hover)
    end
end

"""
    MPCMatrices

A mutable struct that pre-computes and stores the matrices and vectors required for the Model Predictive Control optimization problem.

# Fields
- `P::Matrix{Float64}`: Solution to the Discrete Algebraic Riccati Equation (DARE), used for the terminal cost.
- `U::Matrix{Float64}`: Selection matrix to extract the control inputs over the prediction horizon.
- `H::SparseMatrixCSC{Float64,Int64}`: Hessian matrix of the quadratic programming (QP) problem.
- `b::Vector{Float64}`: Linear term vector of the QP problem.
- `C::SparseMatrixCSC{Float64,Int64}`: Constraint matrix for the system dynamics over the prediction horizon.
- `D::SparseMatrixCSC{Float64,Int64}`: Combined constraint matrix including dynamics and control limits.
- `lb::Vector{Float64}`: Lower bound vector for the QP problem constraints (including initial state and control limits).
- `ub::Vector{Float64}`: Upper bound vector for the QP problem constraints (including initial state and control limits).
- `prob::OSQP.Model`: The OSQP optimization problem object.

# Constructors
- `MPCMatrices(model::Model, state_space::LinearStateSpace, tunning_params::MPCTunningParameters)`: Constructs the `MPCMatrices` object by setting up the QP problem for the MPC. It computes the necessary matrices based on the linearized state-space model, tuning parameters (cost matrices and horizons), and the desired trajectory (used to formulate the cost function).
"""
mutable struct MPCMatrices
    P::Matrix{Float64}
    U::Matrix{Float64}
    H::SparseMatrixCSC{Float64,Int64}
    b::Vector{Float64}
    C::SparseMatrixCSC{Float64,Int64}
    D::SparseMatrixCSC{Float64,Int64}
    lb::Vector{Float64}
    ub::Vector{Float64}
    prob::OSQP.Model

    function MPCMatrices(model, state_space, tunning_params)
        local A = state_space.A_r
        local B = state_space.B_r
        local Q = tunning_params.Q
        local R = tunning_params.R
        local Qn = tunning_params.Qn
        local Nx = state_space.Nx
        local Nu = state_space.Nu
        local Nh = tunning_params.Nh
        local Nc = tunning_params.Nc
        local u_hover = state_space.u_hover

        
        # Compute P from dare (Direct a... Ricatti Equation)
        P = dare(A, B, Q, R) # daRicattiEquation (Pq isso ta aqui??) STUDY THIS!!!
        
        U = kron(Diagonal(I,Nh), [I zeros(Nu,Nx)]) # Matrix that picks out all u
        
        H = sparse([kron(Diagonal(I,Nh-1),[R zeros(Nu,Nx); zeros(Nx,Nu) Q]) zeros((Nx+Nu)*(Nh-1), Nx+Nu); zeros(Nx+Nu,(Nx+Nu)*(Nh-1)) [R zeros(Nu,Nx); zeros(Nx,Nu) P]])
        b = zeros(Nh*(Nx+Nu))
        C = sparse([[B -I zeros(Nx,(Nh-1)*(Nu+Nx))]; zeros(Nx*(Nh-1),Nu) [kron(Diagonal(I,Nh-1), [A B]) zeros((Nh-1)*Nx,Nx)] + [zeros((Nh-1)*Nx,Nx) kron(Diagonal(I,Nh-1),[zeros(Nx,Nu) Diagonal(-I,Nx)])]])

        D = [C; U]

        lb = [zeros(Nx*Nh);
              kron(ones(Nh), model.umin - u_hover)]
 
        ub = [zeros(Nx*Nh);
              kron(ones(Nh), model.umax - u_hover)]

        prob = OSQP.Model()
        OSQP.setup!(prob; P=H, q=b, A=D, l=lb, u=ub, verbose=false, eps_abs=1e-8, eps_rel=1e-8, polish=1);
        
        new(P, U, H, b, C, D, lb, ub, prob)
    end
end

# MPC Controller
"""
    mpc_controller(model::Model, simulation::Simulation, tunning_params::MPCTunningParameters, trajectory::Trajectory, lstate_space::LinearStateSpace, mpc_mats::MPCMatrices, t, x::Vector{Float64}, xref::Vector{Float64})

Computes the optimal control input using Model Predictive Control (MPC).

This function formulates and solves a quadratic programming (QP) problem to find the sequence of future control inputs that minimizes a cost function over the prediction horizon, subject to the system dynamics and control constraints. The first control input in the optimal sequence is then applied to the system.

# Arguments
- `model::Model`: The `Model` struct containing the quadrotor's physical parameters.
- `simulation::Simulation`: The `Simulation` struct containing the simulation parameters.
- `tunning_params::MPCTunningParameters`: The `MPCTunningParameters` struct containing the MPC tuning parameters (prediction horizon, control horizon, cost matrices).
- `trajectory::Trajectory`: The `Trajectory` struct defining the desired motion.
- `lstate_space::LinearStateSpace`: The `LinearStateSpace` struct containing the linearized system dynamics.
- `mpc_mats::MPCMatrices`: The `MPCMatrices` struct containing the pre-computed matrices for the QP problem.
- `t`: The current simulation time [s].
- `x::Vector{Float64}`: The current full state vector of the quadrotor.
- `xref::Vector{Float64}`: The reference state vector over the prediction horizon.

# Returns
- `Vector{Float64}`: The optimal control input vector to be applied at the current time step.
"""
function mpc_controller(
    model::Model,
    simulation::Simulation,
    tunning_params::MPCTunningParameters,
    trajectory::Trajectory, # change name for traj_params
    lstate_space::LinearStateSpace, # linear state space
    mpc_mats::MPCMatrices,
    t,
    x::Vector{Float64},
    xref::Vector{Float64}
    )
    
    Q = tunning_params.Q
    Nh = tunning_params.Nh
    
    Nx = lstate_space.Nx
    Nu = lstate_space.Nu
    
    A_prime = lstate_space.A_r # r stands for reduced system (controlable system)
    B_prime = lstate_space.B_r # r stands for reduced system (controlable system)
    u_hover = lstate_space.u_hover

    P = mpc_mats.P
    b = mpc_mats.b
    lb = mpc_mats.lb
    ub = mpc_mats.ub
    prob = mpc_mats.prob
   
    # Extract Reference Quaternion
    q_ref = xref[4:7]
    q = x[4:7]

    # Ensure that q and q_ref are in the same hemisphere
    # Avoid "windup" phenomenon caused by 4pi periodicity
    if(q'*q_ref < 0)
        q .= -q
    end
    
    x  = [x[1:3]; q; x[8:13]]
    
    x_r = reduce_x_state(x)

    xref_r = zeros(Nx*Nh)
    for i = 1:Nh
        xref_r[((i-1)*Nx + 1):i*Nx] = reduce_x_state(xref[((i-1)*(Nx + 1) + 1):i*(Nx + 1)])
    end
    
    #Update QP problem
    lb[1:Nx] .= -A_prime * x_r
    ub[1:Nx] .= -A_prime * x_r

    # Efetua "rastreamento de referência" 
    for j = 1:(Nh-1)
        b[(Nu+(j-1)*(Nx+Nu)).+(1:Nx)] .= -Q*xref_r[1 + Nx*(j-1):Nx* j]
    end
    b[(Nu+(Nh-1)*(Nx+Nu)).+(1:Nx)] .= -P*xref_r[(Nh*Nx-Nx)+1:Nx*Nh]
    
    OSQP.update!(prob, q=b, l=lb, u=ub)

    #Solve QP
    results = OSQP.solve!(prob)
    Δu = results.x[1:Nu]

    return u_hover + Δu
end