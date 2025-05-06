import Pkg
Pkg.activate(@__DIR__)

using LinearAlgebra
using BlockDiagonals
"""
    QuaternionMatrices()

A struct to hold pre-defined matrices `T` and `H` used in quaternion algebra.

**Fields:**

* `T::Diagonal{Float64, Vector{Float64}}`: A diagonal matrix with the first element as 1.0 and the subsequent three elements as -1.0.
* `H::Matrix{Float64}`: A 4x3 matrix with the first row as zeros and the subsequent 3x3 block as the identity matrix.

**Business Rules:**

* The matrix `T` is a fixed diagonal matrix used in quaternion to rotation matrix conversions.
* The matrix `H` is a fixed matrix used to extract the vector part of a quaternion.
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

#Quaternion stuff
"""
    hat(v)

Computes the skew-symmetric "hat" map of a 3-dimensional vector `v`.
This matrix cross product operator satisfies `hat(v) * w = v × w` for any vectors `v` and `w`.

**Arguments:**

* `v::Vector`: A 3-dimensional vector.

**Returns:**

* `Matrix{Int64}`: A 3x3 skew-symmetric matrix.

**Business Rules:**

* The input `v` must be a vector of length 3.
* The output is a 3x3 matrix where the elements are arranged such that matrix multiplication with another vector is equivalent to the cross product.
"""
function hat(v)
    return [0 -v[3] v[2];
            v[3] 0 -v[1];
            -v[2] v[1] 0]
end

"""
    L(q)

Computes the left-multiplication matrix of a quaternion `q`.
If `q = s + v` where `s` is the scalar part and `v` is the vector part, then `L(q)` is a 4x4 matrix such that `L(q) * p` represents the quaternion product `q * p` for any quaternion `p`.

**Arguments:**

* `q::Vector`: A 4-dimensional quaternion represented as `[s; vx; vy; vz]`.

**Returns:**

* `Matrix{Float64}`: A 4x4 left-multiplication matrix.

**Business Rules:**

* The input `q` must be a vector of length 4.
* The output matrix is constructed based on the scalar and vector parts of the input quaternion.
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
This function uses the left-multiplication matrix `L(q)` and the pre-defined matrices `T` and `H` to compute the rotation matrix.

**Arguments:**

* `q::Vector`: A 4-dimensional unit quaternion.

**Returns:**

* `Matrix{Float64}`: A 3x3 rotation matrix.

**Business Rules:**

* The input `q` is assumed to be a unit quaternion.
* The rotation matrix `Q` is computed using the formula `H' * T * L(q) * T * L(q) * H`.
"""
function qtoQ(q)
    qm = QuaternionMatrices()
    H = qm.H
    T = qm.T
    return H'*T*L(q)*T*L(q)*H
end

"""
    G(q)

Computes the "Attitude Jacobian" matrix `G(q)` for a quaternion `q`.
This matrix is often used in the context of attitude control and estimation.

**Arguments:**

* `q::Vector`: A 4-dimensional quaternion.

**Returns:**

* `Matrix{Float64}`: A 4x3 "Attitude Jacobian" matrix.

**Business Rules:**

* The output matrix `G` is calculated as the product of the left-multiplication matrix `L(q)` and the matrix `H`.
"""
function G(q) # "Attitude Jacobian"
    qm = QuaternionMatrices()
    H = qm.H

    G = L(q)*H
end

"""
    rptoq(ϕ)

Converts a Rodriguez parameter vector `ϕ` to a unit quaternion `q`.
The Rodriguez parameters provide a minimal representation of rotation.

**Arguments:**

* `ϕ::Vector`: A 3-dimensional Rodriguez parameter vector.

**Returns:**

* `Vector{Float64}`: A 4-dimensional unit quaternion.

**Business Rules:**

* The input `ϕ` must be a vector of length 3.
* The output quaternion `q` is normalized to be a unit quaternion.
* The conversion formula used is `(1/sqrt(1+ϕ'*ϕ)) * [1; ϕ]`.
"""
function rptoq(ϕ) # Rodriguez Parameter to Quaternion
    (1/sqrt(1+ϕ'*ϕ))*[1; ϕ]
end

"""
    qtorp(q)

Converts a unit quaternion `q` to a Rodriguez parameter vector `ϕ`.
This function provides the inverse transformation of `rptoq`.

**Arguments:**

* `q::Vector`: A 4-dimensional unit quaternion with the scalar part `q[1]`.

**Returns:**

* `Vector{Float64}`: A 3-dimensional Rodriguez parameter vector.

**Business Rules:**

* The input `q` must be a unit quaternion of length 4.
* The scalar part of the quaternion `q[1]` must be non-zero.
* The conversion formula used is `q[2:4] / q[1]`.
"""
function qtorp(q) # Quaternion to Rodriguez parameter
    q[2:4]/q[1]
end

# Since the unit quaternion has unit norm, is not possible to drive it to the origin ("control it")
"""
    E(q)

Constructs a block diagonal matrix `E(q)` to reduce the state space, particularly for systems involving unit quaternions, making the system controllable.
Due to the unit norm constraint of quaternions, they cannot be directly driven to the origin. This matrix projects the state onto a controllable subspace.

**Arguments:**

* `q::Vector`: A 4-dimensional quaternion.

**Returns:**

* `BlockDiagonal{Float64, Matrix{Float64}}`: A block diagonal matrix. The blocks are a 3x3 identity matrix, the 4x3 attitude Jacobian `G(q)`, and a 6x6 identity matrix.

**Business Rules:**

* The output matrix `E` is constructed by combining identity matrices and the attitude Jacobian `G(q)`.
* The structure of `E` suggests it's designed for a state vector that includes at least an attitude representation (potentially using quaternions or related variables) and other system states.
* The use of `BlockDiagonal` indicates that the matrix has a specific block structure that might simplify certain operations or analyses.
"""
function E(q) # Matrix to reduce state and turn the system controlable
    E = BlockDiagonal([1.0*I(3), G(q), 1.0*I(6)])
end