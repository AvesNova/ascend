using OrdinaryDiffEq
using CliffordAlgebras
using StaticArrays
import("pga.jl")

mutable struct PhysicsComponent
    twist::MVector{6,Float64}
    pose::MVector{8,Float64}
    inertia::MultiVector
    forque::Function
end

"""
    inertia_map(twist::MultiVector, inertia::MultiVector) -> MultiVector

Compute the momentum of a given twist and inertia.

# Arguments
- `twist::MultiVector`: A PGA Line representing the twist (linear + angular velocity).
- `inertia::MultiVector`: A PGA Line representing the moment of inertia.

# Returns
- `MultiVector`: A PGA Line indicating the momentum.

# Example
'''julia
twist = pga_line(0.0, 0.0, 0.0, 0.1, 0.001, 0.0)
inertia = pga_line(2, 1, 3, 1, 1, 1)
inertia_map(twist, inertia)
'''
"""
function inertia_map(twist::MultiVector, inertia::MultiVector)::MultiVector
    return dual(twist) .* inertia
end

"""
    inv_inertia_map(momentum::MultiVector, inertia::MultiVector) -> MultiVector

Calculate the twist of a given momentum and moment of inertia.

# Arguments

- `momentum::MultiVector`: A PGA Line representing the momentum.
- `inertia::MultiVector`: A PGA Line representing the inertia.

# Returns

- `MultiVector`: The PGA Line representing the twist (linear + angular velocity).

# Example

'''julia
julia> momentum = pga_line(0.0, 0.001, 0.3, 0.0, 0.0, 0.0)
julia> inertia = pga_line(2, 1, 3, 1, 1, 1)
julia> inv_inertia_map(momentum, inertia)
'''
"""
function inv_inertia_map(momentum::MultiVector, inertia::MultiVector)::MultiVector
    return dual(momentum ./ inertia)
end

"""
    Δpose!(Δpose::Vector, twist::Vector, pose::Vector, p, t::Real)

Updates the `Δpose` vector based on the given `twist` and `pose`.
"""
function Δpose!(
    Δpose::V{T}, 
    twist::V{T}, 
    pose::V{T}, p, t::Real
)::Nothing where {T <: Real, V <: AbstractVector}
    twist_line::MultiVector = pga_line(twist)
    pose_motor::MultiVector = pga_motor(pose)

    Δpose_line::MultiVector = -0.5 * pose_motor * twist_line
    Δpose .= coefficients(Δpose_line, PGA_MOTOR_INDICES_MVECTOR)
    return nothing
end

"""
    Δtwist!(Δtwist::Vector, twist::Vector, pose::Vector, p, t::Real)::MultiVector

Acceleration = I⁻¹[Forque + Rate ×₋ I[Rate]]
"""
function Δtwist!(
    Δtwist::V{T}, 
    twist::V{T}, 
    pose::V{T}, 
    p, t::Real
)::Nothing where {T <: Real, V <: AbstractVector}
    inertia::MultiVector = p.inertia
    twist_line::MultiVector = pga_line(twist)

    Itwist::MultiVector = inertia_map(twist_line, inertia)
    momentum::MultiVector = twist_line ×₋ Itwist + p.forque(twist, pose, t)
    Δtwist_line::MultiVector = inv_inertia_map(momentum, inertia)

    Δtwist .= coefficients(Δtwist_line, PGA_LINE_INDICES_MVECTOR)
    return nothing
end

"""
    step(twist::Vector, pose::Vector, inertia::MultiVector, Δt::Real; forque::Function) -> Tuple{Vector, Vector}

Perform one step of the rigid body motion simulation without modifying the input vectors.

# Arguments
- `twist::Vector`: Initial twist coefficients.
- `pose::Vector`: Initial pose coefficients.
- `inertia::MultiVector`: Moment of inertia as a PGA Line.
- `Δt::Real`: Time step for the simulation.
- `forque::Function`: (Optional) A function returning external forque (force + torque) as a PGA Line. Defaults to zero forque.

# Returns
- A tuple containing the resulting twist and pose vectors after the simulation step.
"""
function rbm_physics_step(
    twist::V,
    pose::V,
    inertia::MultiVector,
    Δt::Float64;
    forque::Function=((twist, pose, t) -> pga_line(0,0,0,0,0,0)),
    alg=Tsit5,
    kwargs...,
)::Tuple{V, V} where {V <: AbstractVector}
    pose_motor::MultiVector = pga_motor(pose)
    pose_motor /= norm(pose_motor)
    pose = coefficients(pose_motor, PGA_MOTOR_INDICES_MVECTOR)

    p = (inertia=inertia, forque=forque)
    rbm_prob = DynamicalODEProblem(Δtwist!, Δpose!, twist, pose, (0.0, Δt), p; kwargs...)
    sol = solve(rbm_prob, alg())

    return sol[end].x[1], sol[end].x[2]
end

"""
    step(phx_component::PhysicsComponent, ) -> Tuple{MultiVector, MultiVector}

Perform one step of the rigid body motion simulation.
"""
function rbm_physics_step!(
    phx_component::PhysicsComponent,
    Δt::Float64;
)
    (;twist, pose, inertia, forque) = phx_component

    pose_motor = pga_motor(pose)
    pose_motor /= norm(pose_motor)
    pose = coefficients(pose_motor, PGA_MOTOR_INDICES_MVECTOR)

    p = (inertia=inertia, forque=forque)
    rbm_prob = DynamicalODEProblem(Δtwist!, Δpose!, twist, pose, (0.0, Δt), p)
    sol = solve(rbm_prob, Tsit5())

    twist = sol[end].x[1]
    pose = sol[end].x[2]

    phx_component.twist = twist
    phx_component.pose = pose
end
