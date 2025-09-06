import Pkg
Pkg.activate(@__DIR__)

using LinearAlgebra
using SparseArrays
using ForwardDiff
using ControlSystems
using OSQP

include("Quaternion.jl")
include("MPCTunningParams.jl")

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
            dynamics_rk4(x, u_hover, (x,u)->quad_dynamics(model, x, u, height, [1.0; 0.0; 0.0], 0.0), simulation.h_controller)
            ,x_hover)
        B = ForwardDiff.jacobian(u ->
            dynamics_rk4(x_hover, u, (x,u)->quad_dynamics(model, x, u, height, [1.0; 0.0; 0.0], 0.0), simulation.h_controller)
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
        local Nx = state_space.Nx
        local Nu = state_space.Nu
        local Nh = tunning_params.Nh
        local Nc = tunning_params.Nc
        local u_hover = state_space.u_hover

        
        # Compute P from dare (Direct Algebraic Ricatti Equation)
        P = dare(A, B, Q, R) # Effectivelly the Qn! Terminal Cost
        
        U = kron(Diagonal(I,Nh), [I zeros(Nu,Nx)]) # Matrix that picks out all u
        Θ = kron(Diagonal(I,Nh), [ 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0;
                                   0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0;
                                   0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0]) #Matrix that picks out all x3 (θ)
        H = sparse([kron(Diagonal(I,Nh-1),[R zeros(Nu,Nx); zeros(Nx,Nu) Q]) zeros((Nx+Nu)*(Nh-1), Nx+Nu); zeros(Nx+Nu,(Nx+Nu)*(Nh-1)) [R zeros(Nu,Nx); zeros(Nx,Nu) P]])
        b = zeros(Nh*(Nx+Nu))
        C = sparse([[B -I zeros(Nx,(Nh-1)*(Nu+Nx))]; zeros(Nx*(Nh-1),Nu) [kron(Diagonal(I,Nh-1), [A B]) zeros((Nh-1)*Nx,Nx)] + [zeros((Nh-1)*Nx,Nx) kron(Diagonal(I,Nh-1),[zeros(Nx,Nu) Diagonal(-I,Nx)])]])

        n_restrictions = Nh - Nc - 1
        n_fixed_control_inputs = Nh - Nc

        # índices e valores para o +1 na diagonal
        r1 = 1:n_restrictions
        c1 = 1:n_restrictions
        v1 = ones(n_restrictions)

        # índices e valores para o -1 na sub-diagonal
        r2 = 1:n_restrictions
        c2 = 2:n_restrictions+1
        v2 = -ones(n_restrictions)

        F = sparse( vcat(r1, r2), vcat(c1, c2), vcat(v1, v2), n_restrictions, n_fixed_control_inputs)
        
        O = hcat(zeros(Nu*(Nh-Nc-1),Nc*Nu),  kron(F, I(Nu)))
        # W é a matriz que define o que se entende por Horizonte de Controle
        W = O*U

        
        #D = [C; U; W]
        #D = [C; U; Θ]
        D = [C; U]

        lb = [zeros(Nx*Nh);
              kron(ones(Nh), model.umin - u_hover)]
              #kron(ones(Nh), [ -0.13165249758739583; -0.13165249758739583; -1.0])]
              #zeros(Nu*(n_restrictions))]
 
        ub = [zeros(Nx*Nh);
              kron(ones(Nh), model.umax - u_hover)]
              #kron(ones(Nh), [ 0.13165249758739583; 0.13165249758739583; 1.0])]
              #zeros(Nu*(n_restrictions))]

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

    # AAAA, vamos tentar mudar essa questao..
    
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

    #println(results.x[Nu + 4:Nu + 6])
    #vector = kron(Diagonal(I,Nh), [ 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0;
    #                               0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0;
    #                               0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0]) * results.x

    #lower = kron(ones(Nh), [ -1.0; -1.0; -1.0])
    #upper = kron(ones(Nh), [ 1.0; 1.0; 1.0])
    #ok_mask = (vector .>= lower) .& (vector .<= upper)

    #viol_lower = findall(vector .< lower)
    #viol_upper = findall(vector .> upper)

    #println(viol_lower)
    #println(viol_upper)
    
    # Check solver status
    #if results.info.status != :Solved
    #    println(results.info.status)
    #    error("OSQP did not solve the problem!")
    #end
    
    Δu = results.x[1:Nu]

    return u_hover + Δu
end