"""
    MPCTunningParameters

A mutable struct to hold the tuning parameters for the Model Predictive Controller (MPC).

# Fields
- `Nh::Int64`: Predictive horizon - the number of future time steps the controller considers.
- `Nc::Int64`: Control horizon - the number of future time steps over which the control inputs are optimized (Nc <= Nh).
- `Q::Matrix{Float64}`: Stage State Cost Penalty - a positive semi-definite matrix penalizing deviations from the desired state at each step within the prediction horizon.
- `R::Matrix{Float64}`: Stage Control Cost Penalty - a positive definite matrix penalizing the control effort at each step within the control horizon.
"""
mutable struct MPCTunningParameters
    Nh::Int64           # Predictive horizon
    Nc::Int64           # Control horizon (Nc < Nh)
    Q::Matrix{Float64}  # Stage State Cost Penalty
    R::Matrix{Float64}  # Stage Control Cost Penalty
end