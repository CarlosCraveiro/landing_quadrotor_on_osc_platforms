import Pkg
Pkg.activate(@__DIR__)

using LinearAlgebra
using BlockDiagonals

"""
    QuaternionMatrices()

Constructs a structure containing two matrices used in quaternion operations.

`T`: A 4x4 diagonal matrix with `[1; -1; -1; -1]` on its diagonal, acting as a metric signature for quaternion space in matrix operations.
`H`: A 4x3 matrix that selects the vector part of a quaternion when used in matrix multiplications.

# Returns
- `QuaternionMatrices`: A struct with fields `T` (a 4x4 diagonal matrix) and `H` (a 4x3 matrix).
"""
struct QuaternionMatrices
    T::Diagonal{Float64, Vector{Float64}}
    H::Matrix{Float64}
    function QuaternionMatrices()
        local T = Diagonal([1; -ones(3)])
        local H = [zeros(1,3); I]

        new(T, H)
    end
end


"""
    hat(v)

Computes the skew-symmetric matrix of a 3D vector.

This matrix representation allows the cross product with `v` to be expressed as a matrix multiplication.

# Arguments
- `v`: A 3-element vector.

# Returns
- `Matrix{Float64}`: A 3x3 skew-symmetric matrix.
"""
function hat(v)
    return [0 -v[3] v[2];
            v[3] 0 -v[1];
            -v[2] v[1] 0]
end

"""
    L(q)

Computes the left quaternion multiplication matrix for a quaternion `q`.

Given a quaternion `q = s + v`, where `s` is the scalar part and `v` is the vector part, this function returns the matrix `L(q)` such that `L(q) * p` represents the quaternion product `q * p`, where `p` is another quaternion.

# Arguments
- `q`: A 4-element vector representing a quaternion (scalar part first).

# Returns
- `Matrix{Float64}`: A 4x4 matrix representing the left quaternion multiplication.
"""
function L(q)
    s = q[1]
    v = q[2:4]
    L = [s    -v';
         v  s*I+hat(v)]
    return L
end

"""
    qtoQ(q)

Converts a quaternion `q` to a rotation matrix `Q`.

    `Q = H'*T*L(q)*T*L(q)*H`

This function uses the `QuaternionMatrices` struct and the `L` function to perform the conversion.  It effectively maps a quaternion to its equivalent rotation matrix representation.

# Arguments
- `q`: A 4-element vector representing a quaternion (scalar part first).

# Returns
- `Matrix{Float64}`: A 3x3 rotation matrix.
"""
function qtoQ(q)
    qm = QuaternionMatrices()
    H = qm.H
    T = qm.T
    return H'*T*L(q)*T*L(q)*H
end

"""
    G(q)

Computes the Attitude Jacobian matrix for a quaternion.

This matrix relates changes in the quaternion's tangent space to changes in the quaternion itself.

# Arguments
- `q`: A 4-element vector representing a quaternion (scalar part first).

# Returns
- `Matrix{Float64}`: A 4x3 Attitude Jacobian matrix.
"""
function G(q)
    qm = QuaternionMatrices()
    H = qm.H

    G = L(q)*H
end

"""
    rptoq(ϕ::Vector{Float64})

Converts a Rodriguez parameter vector to a quaternion.

Rodriguez parameters are an alternative way to represent rotations.

# Arguments
- `ϕ::Vector{Float64}`: A 3-element vector representing the Rodriguez parameters.

# Returns
- `Vector{Float64}`: A 4-element vector representing the equivalent quaternion (scalar part first).
"""
function rptoq(ϕ::Vector{Float64}) # Rodriguez Parameter to Quaternion
    (1/sqrt(1+ϕ'*ϕ))*[1; ϕ]
end

"""
    qtorp(q)

Converts a quaternion to its corresponding Rodriguez parameter vector.

This is the inverse operation of converting Rodriguez parameters to a quaternion.

# Arguments
- `q`: A 4-element vector representing a quaternion (scalar part first).

# Returns
- `Vector{Float64}`: A 3-element vector representing the Rodriguez parameters.
"""
function qtorp(q) # Quaternion to Rodriguez parameter
    q[2:4]/q[1]
end


"""
    E(q)

Constructs a block diagonal matrix for state reduction in control systems.

This matrix, built using the Attitude Jacobian, can be used to transform the state space, turning the system controllable.

# Arguments
- `q`: A 4-element vector representing a quaternion (scalar part first).

# Returns
- `BlockDiagonal{Float64, Matrix{Float64}}`: A block diagonal matrix.
"""
function E(q) # Matrix to reduce state and turn the system controlable
    E = BlockDiagonal([1.0*I(3), G(q), 1.0*I(6)])
end