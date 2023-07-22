using OrdinaryDiffEq

include("pga.jl")

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
twist = LinePGA(0.0, 0.0, 0.0, 0.1, 0.001, 0.0)
inertia = LinePGA(2, 1, 3, 1, 1, 1)
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
julia> momentum = LinePGA(0.0, 0.001, 0.3, 0.0, 0.0, 0.0)
julia> inertia = LinePGA(2, 1, 3, 1, 1, 1)
julia> inv_inertia_map(momentum, inertia)
'''
"""
function inv_inertia_map(momentum::MultiVector, inertia::MultiVector)::MultiVector
    return dual(momentum ./ inertia)
end

"""
    Δpose(twist::MultiVector, pose::MultiVector) -> MultiVector

Calculate the change in pose (position + orientation) for a given twist (linear + angular velocity).

# Arguments

- `twist::MultiVector`: A PGA Line representing the twist (linear + angular velocity).
- `pose::MultiVector`: A PGA Motor representing the current pose (position + orientation).

# Returns

- `MultiVector`: The change in the pose.

# Example

'''julia
julia> twist = LinePGA(0.0, 0.0, 0.0, 0.1, 0.001, 0.0)
julia> pose = MotorPGA(-1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 3.0, -2.0)
julia> Δpose(twist, pose)
'''
"""
function Δpose(twist::MultiVector, pose::MultiVector)::MultiVector
    return -0.5 * pose * twist
end

"""
    Δpose!(Δpose::Vector, twist::Vector, pose::Vector, p, t::Real)

Updates the `Δpose` vector based on the given `twist` and `pose`.
"""
function Δpose!(Δpose::Vector, twist::Vector, pose::Vector, p, t::Real)
    twist_line::MultiVector = LinePGA(twist)
    pose_motor::MultiVector = MotorPGA(pose)
    Δpose_line::MultiVector = -0.5 * pose_motor * twist_line
    Δpose .= coefficients(Δpose_line, MOTOR_PGA_INDICES_VECTOR)
end

"""
    Δtwist(twist::MultiVector, inertia::MultiVector; forque::MultiVector = LinePGA(0,0,0,0,0,0)) -> MultiVector

Calculate the change in twist given inertia and an optional external forque (force + torque).

Acceleration is computed as I⁻¹[Forque + Twist ×₋ I[Twist]].

# Arguments

- `twist::MultiVector`: A line representing the twist (linear + angular velocity).
- `inertia::MultiVector`: A line representing the inertia.
- `forque::MultiVector` (optional): An external force represented as a line. Defaults to zero.

# Returns

- `MultiVector`: The change in the twist.

# Example

'''julia
julia> twist = LinePGA(0.0, 0.0, 0.0, 0.1, 0.001, 0.0)
julia> inertia = LinePGA(2, 1, 3, 1, 1, 1)
julia> Δtwist(twist, inertia)
'''
"""
function Δtwist(
    twist::MultiVector, 
    inertia::MultiVector; 
    forque::MultiVector = LinePGA(0,0,0,0,0,0),
)::MultiVector

    Itwist::MultiVector = inertia_map(twist, inertia)
    momentum::MultiVector = forque + twist ×₋ Itwist
    return inv_inertia_map(momentum, inertia)
end

"""
    Δtwist!(Δtwist::Vector, twist::Vector, pose::Vector, p, t::Real)::MultiVector

Acceleration = I⁻¹[Forque + Rate ×₋ I[Rate]]
"""
function Δtwist!(Δtwist::Vector, twist::Vector, pose::Vector, p, t::Real)
    inertia::MultiVector = p.inertia
    twist_line::MultiVector = LinePGA(twist)

    Itwist::MultiVector = inertia_map(twist_line, inertia)
    momentum::MultiVector = twist_line ×₋ Itwist + p.forque(twist, pose, t)
    Δtwist_line::MultiVector = inv_inertia_map(momentum, inertia)

    Δtwist .= coefficients(Δtwist_line, LINE_PGA_INDICES_VECTOR)
end

"""
    euler_step(twist::MultiVector, pose::MultiVector, inertia::MultiVector, step_count::Int, dt::Real) -> Tuple{MultiVector, MultiVector}

Perform an Euler integration step on the pose and twist.

# Arguments

- `twist::MultiVector`: A PGA Line representing the twist.
- `pose::MultiVector`: A PGA Motor representing the current pose.
- `inertia::MultiVector`: A PGA Line representing the inertia.
- `step_count::Int`: Number of steps in the integration.
- `dt::Real`: Time step size.

# Returns

- `Tuple{MultiVector, MultiVector}`: The updated twist and pose after performing the Euler integration.

# Example

'''julia-repl
julia> twist = LinePGA(0.0, 0.0, 0.0, 0.1, 0.001, 0.0)
julia> pose = MotorPGA(-1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 3.0, -2.0)
julia> inertia = LinePGA(2, 1, 3, 1, 1, 1)
julia> step_count = 1000
julia> dt = 1.0
julia> euler_step(twist, pose, inertia, step_count, dt)
'''
"""
function euler_step(twist::MultiVector, pose::MultiVector, inertia::MultiVector, step_count::Int, dt::Real)::Tuple{MultiVector, MultiVector}
    for _ in 1:step_count
        twist += Δtwist(twist, inertia) * dt / step_count
        pose += Δpose(twist, pose) * dt / step_count
    end
    pose /= norm(pose)

    return twist, pose
end

"""
    init_rigid_body_motion(twist::Vector, pose::Vector, inertia::MultiVector; forque::Function)::SciMLBase.DynamicalODEProblem

Initialize a dynamical ODE problem for rigid body motion.

# Arguments
- `twist::Vector`: Initial twist coefficients.
- `pose::Vector`: Initial pose coefficients.
- `inertia::MultiVector`: Moment of inertia as a PGA Line.
- `forque::Function`: A function returning external force/torque as a PGA Line (defaults to a zero function).

# Returns
- A `DynamicalODEProblem` for the given parameters.
"""
function RigidBodyMotionProblem(
    twist::Vector,
    pose::Vector,
    inertia::MultiVector,
    Δt::Real;
    forque::Function=((pose, t) -> LinePGA(0,0,0,0,0,0))
)
    pose_motor::MultiVector = MotorPGA(pose)
    pose_motor /= norm(pose_motor)
    pose = coefficients(pose_motor, MOTOR_PGA_INDICES_VECTOR)

    p = (inertia=inertia, forque=forque)
    return DynamicalODEProblem(Δtwist!, Δpose!, twist, pose, (0.0, Δt), p)
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
    twist::Vector,
    pose::Vector,
    inertia::MultiVector,
    Δt::Real;
    forque::Function=((twist, pose, t) -> LinePGA(0,0,0,0,0,0))
)::Tuple{Vector{Float64}, Vector{Float64}}
    pose_motor::MultiVector = MotorPGA(pose)
    pose_motor /= norm(pose_motor)
    pose = coefficients(pose_motor, MOTOR_PGA_INDICES_VECTOR)

    p = (inertia=inertia, forque=forque)
    rbm_prob = DynamicalODEProblem(Δtwist!, Δpose!, twist, pose, (0.0, Δt), p)
    sol = solve(rbm_prob, Tsit5())

    return sol[end].x[1], sol[end].x[2]
end

# function rbm_physics_step(
#     twist::MultiVector,
#     pose::MultiVector,
#     inertia::MultiVector,
#     Δt::Real;
#     forque::Function=((twist, pose, t) -> LinePGA(0,0,0,0,0,0))
# )::Tuple{MultiVector, MultiVector}
#     pose /= norm(pose)

#     twist_vector = coefficients(twist, LINE_PGA_INDICES_VECTOR)
#     pose_vector = coefficients(twist, MOTOR_PGA_INDICES_VECTOR)

#     p = (inertia=inertia, forque=forque)
#     rbm_prob = DynamicalODEProblem(Δtwist!, Δpose!, twist_vector, pose_vector, (0.0, Δt), p)
#     sol = solve(rbm_prob, Tsit5())

#     twist = LinePGA(sol[end].x[1])
#     pose = MotorPGA(sol[end].x[2])

#     return twist, pose
# end
