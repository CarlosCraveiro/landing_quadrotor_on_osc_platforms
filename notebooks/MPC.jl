import Pkg
Pkg.activate(@__DIR__)

using LinearAlgebra
using SparseArrays
using ForwardDiff
using ControlSystems
using OSQP

mutable struct MPCTunningParameters
    Nh::Int64           # Predictive horizon
    Nc::Int64           # Control horizon (Nc < Nh)
    Q::Matrix{Float64}  # Stage State Cost Penalty
    R::Matrix{Float64}  # Stage Control Cost Penalty
    Qn::Matrix{Float64} # Terminal State Cost Penalty
end

"""
    TODO...
"""
struct LinearStateSpace
    Nx::Int64          # State vector dimension
    Nu::Int64          # Control vector dimension
    A::Matrix{Float64} # State Matrix
    B::Matrix{Float64} # Control Matrix
    x_hover::Vector{Float64}   # x value for which the linearization was made
    u_hover::Vector{Float64}   # u value for which the linearization was made

    function LinearStateSpace(model, simulation, Nx, Nu, height)
        x_hover, u_hover = find_hover_conditions(model, height)
        A = ForwardDiff.jacobian(x->
            dynamics_rk4(x, u_hover, (x,u)->quad_dynamics(model, x, u, height), simulation)
            ,x_hover)
        B = ForwardDiff.jacobian(u ->
            dynamics_rk4(x_hover, u, (x,u)->quad_dynamics(model, x, u, height), simulation)
        , u_hover)

        new(Nx, Nu, A, B, x_hover, u_hover)
    end
end

"""
TODO...
"""
mutable struct MPCMatrices
    P::Matrix{Float64}
    U::Matrix{Float64}
    Θ::Matrix{Float64}
    H::SparseMatrixCSC{Float64,Int64}
    b::Vector{Float64}
    C::SparseMatrixCSC{Float64,Int64}
    Y::Matrix{Float64}
    D::SparseMatrixCSC{Float64,Int64}
    lb::Vector{Float64}
    ub::Vector{Float64}

    """
        trajectory is temporarily here because i'm trying to make the problem linear again...
    """
    function MPCMatrices(model, state_space, tunning_params, trajectory)
        A = state_space.A
        B = state_space.B
        Q = tunning_params.Q
        R = tunning_params.R
        Qn = tunning_params.Qn
        Nx = state_space.Nx
        Nu = state_space.Nu
        Nh = tunning_params.Nh
        Nc = tunning_params.Nc
        x_hover = state_space.x_hover
        u_hover = state_space.u_hover

        # Compute P from dare (Direct a... Ricatti Equation)
        P = dare(A, B, Q, R) # daRicattiEquation (Pq isso ta aqui??) STUDY THIS!!!
        
        U = kron(Diagonal(I,Nh), [I zeros(Nu,Nx)]) # Matrix that picks out all u
        Θ = kron(Diagonal(I,Nh), [0 0 0 0 1 0 0 0]) # Matrix that picks out all x3 (θ) ... explain this better TODO!!
        H = sparse([kron(Diagonal(I,Nh-1),[R zeros(Nu,Nx); zeros(Nx,Nu) Q]) zeros((Nx+Nu)*(Nh-1), Nx+Nu); zeros(Nx+Nu,(Nx+Nu)*(Nh-1)) [R zeros(Nu,Nx); zeros(Nx,Nu) P]])
        b = zeros(Nh*(Nx+Nu))
        C = sparse([[B -I zeros(Nx,(Nh-1)*(Nu+Nx))]; zeros(Nx*(Nh-1),Nu) [kron(Diagonal(I,Nh-1), [A B]) zeros((Nh-1)*Nx,Nx)] + [zeros((Nh-1)*Nx,Nx) kron(Diagonal(I,Nh-1),[zeros(Nx,Nu) Diagonal(-I,Nx)])]])
        # Cria uma matriz que seleciona o 4º elemento do bloco (ou seja, o y)
        Y = kron(Diagonal(I, Nh), [0 0 0 1 0 0 0 0])
        D = [C; U; Θ; Y]
        #local x_hover, u_hover = find_hover_conditions(model, temp_height) # Solve this problem TODO!

        lb = [zeros(Nx*Nh);
              kron(ones(Nh), model.umin - u_hover);
              -0.2 * ones(Nh);
              -4.0 * ones(Nh)]
        ub = [zeros(Nx*Nh);
              kron(ones(Nh), model.umax - u_hover);
              0.2 * ones(Nh);
              4.0 * ones(Nh)]
        new(P, U, Θ, H, b, C, Y, D, lb, ub)
    end
end

# MPC Controller
function mpc_controller(model, simulation, tunning_params, trajectory, t, x, xref)
    Q = tunning_params.Q
    Nh = tunning_params.Nh
    Nu = 2
    Nx = 6
    #height = x[2] - (xref[2] -  trajectory.stat_height)
    #lstate_space = LinearStateSpace(model, simulation, Nx, Nu, height)

    lstate_space = LinearStateSpace(model, simulation, Nx, Nu, trajectory.stat_height)

    A_prime = lstate_space.A
    B_prime = lstate_space.B
    x_hover = lstate_space.x_hover
    u_hover = lstate_space.u_hover

    # ....
    mpc_mats = MPCMatrices(model, lstate_space, tunning_params, trajectory)

    P = mpc_mats.P
    H = mpc_mats.H
    b = mpc_mats.b
    D = mpc_mats.D
    lb = mpc_mats.lb
    ub = mpc_mats.ub

    #Update QP problem
    lb[1:6] .= -A_prime * x
    ub[1:6] .= -A_prime * x

    # Atualiza os limites para a restrição Y
    # Índices para Y: (Nx*Nh + 2*Nh + 1) até (Nx*Nh + 3*Nh)

    #y_indices = (Nx*Nh + 3*Nh + 1):(Nx*Nh + 4*Nh)
    #lb[y_indices] .= xref[Nx + 2]

    #println(xref[Nh*Nx-4])
    # Efetua \"rastreamento de referência
    # Cte se xref não mudar
    
    for j = 1:(Nh-1)
        b[(Nu+(j-1)*(Nx+Nu)).+(1:Nx)] .= -Q*xref[1 + Nx*(j-1):Nx* j]
        #b[(Nu+(j-1)*(Nx+Nu)).+(1:Nx)] .= -Q*xref[1:Nx]
    end
    b[(Nu+(Nh-1)*(Nx+Nu)).+(1:Nx)] .= -P*xref[(Nh*Nx-Nx)+1:Nx*Nh]
    #b[(Nu+(Nh-1)*(Nx+Nu)).+(1:Nx)] .= -P*xref[1:Nx]
    #OSQP.update!(prob, q=b, l=lb, u=ub)
    prob = OSQP.Model()
    OSQP.setup!(prob; P=H, q=b, A=D, l=lb, u=ub, verbose=false, eps_abs=1e-8, eps_rel=1e-8, polish=1);

    #Solve QP
    results = OSQP.solve!(prob)
    Δu = results.x[1:Nu]
    #println(results.x)

    return u_hover + Δu
end