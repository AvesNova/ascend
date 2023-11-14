using OrdinaryDiffEq
using CliffordAlgebras
import CliffordAlgebras: PGA_MOTOR_INDICES_MVECTOR, PGA_LINE_INDICES_MVECTOR
using StaticArrays
using Overseer



"""
    inertia_map(twist::MultiVector, inertia::MultiVector) -> MultiVector

Compute the momentum of a given twist and inertia.

# Arguments
- `twist::MultiVector`: A PGA Line representing the twist (linear + angular velocity).
- `inertia::MultiVector`: A PGA Line representing the moment of inertia.

# Returns
- `MultiVector`: A PGA Line indicating the momentum.

# Example
```julia
twist = pga_line(0.0, 0.0, 0.0, 0.1, 0.001, 0.0)
inertia = pga_line(2, 1, 3, 1, 1, 1)
inertia_map(twist, inertia)
```
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

```julia
julia> momentum = pga_line(0.0, 0.001, 0.3, 0.0, 0.0, 0.0)
julia> inertia = pga_line(2, 1, 3, 1, 1, 1)
julia> inv_inertia_map(momentum, inertia)
```
"""
function inv_inertia_map(momentum::MultiVector, inertia::MultiVector)::MultiVector
    return dual(momentum ./ inertia)
end

"""
    Δpose!(Δpose::AbstractVector, twist::AbstractVector, pose::AbstractVector, p, t::Real) -> Nothing

Updates the `Δpose` vector based on the given `twist` and `pose`.

# Arguments
- `Δpose::AbstractVector`: The vector to be updated.
- `twist::AbstractVector`: The twist vector.
- `pose::AbstractVector`: The pose vector.
- `p`: The parameters for the ODE.
- `t::Real`: The time variable.

# Returns
- `Nothing`
"""
function Δpose!(
    Δpose::AbstractVector,
    twist::AbstractVector,
    pose::AbstractVector,
    p, 
    t::Real
)::Nothing
    twist_line::MultiVector = pga_line(twist)
    pose_motor::MultiVector = pga_motor(pose)

    Δpose_line::MultiVector = -0.5 * pose_motor * twist_line
    Δpose .= coefficients(Δpose_line, PGA_MOTOR_INDICES_MVECTOR)
    return nothing
end

function Δpose(twist::MultiVector, pose::MultiVector)::MultiVector
    return -0.5 * pose * twist
end

"""
    Δtwist!(Δtwist::AbstractVector, twist::AbstractVector, pose::AbstractVector, p, t::Real) -> Nothing

Acceleration = Δtwist = I⁻¹[Twist ×₋ I[Twist] + Forque]
Computes the acceleration of a rigid body given its twist, pose, and external forces.

# Arguments
- `Δtwist::AbstractVector`: The vector to be updated.
- `twist::AbstractVector`: The twist vector.
- `pose::AbstractVector`: The pose vector.
- `p`: The parameters for the ODE.
- `t::Real`: The time variable.

# Returns
- `Nothing`
"""
function Δtwist!(
    Δtwist::AbstractVector,
    twist::AbstractVector,
    pose::AbstractVector,
    p,
    t::Real
)::Nothing
    inertia::MultiVector = p.inertia
    twist_line::MultiVector = pga_line(twist)

    Itwist::MultiVector = inertia_map(twist_line, inertia)
    momentum::MultiVector = twist_line ×₋ Itwist + p.forque(twist, pose, p.button_controls, p.axis_controls, t)
    Δtwist_line::MultiVector = inv_inertia_map(momentum, inertia)

    Δtwist .= coefficients(Δtwist_line, PGA_LINE_INDICES_MVECTOR)
    return nothing
end

function Δtwist(
    twist::MultiVector,
    pose::MultiVector,
    p, 
    t::Real
)::MultiVector
    Itwist::MultiVector = inertia_map(twist, p.inertia)
    momentum::MultiVector = twist ×₋ Itwist + p.forque(twist, pose, p.button_controls, p.axis_controls, t)
    Δtwist::MultiVector = inv_inertia_map(momentum, p.inertia)
    return Δtwist
end

function Δtwist_kinematic(twist::MultiVector)::MultiVector
    return dual(twist ×₋ dual(twist))
end

"""
    kinetic_step(
        twist::AbstractVector,
        pose::AbstractVector;
        inertia::MultiVector = pga_line(1,1,1,1,1,1),
        forque::Function = (args...; kwargs...) -> pga_line(0,0,0,0,0,0),
        axis_controls::Union{Vector, MVector} = MVector{0,Float64}(),
        button_controls::Union{Vector, MVector} = MVector{0,Float64}(),
        Δt::Float64 = 1.0,
    )::Tuple{AbstractVector, AbstractVector}

Performs one step of the rigid body motion simulation without modifying the input vectors.

# Arguments
- `twist::AbstractVector`: Initial twist coefficients.
- `pose::AbstractVector`: Initial pose coefficients.
- `inertia::MultiVector`: Moment of inertia as a PGA Line.
- `Δt::Real`: Time step for the simulation.
- `forque::Function`: (Optional) A function returning external forque (force + torque) as a PGA Line. Defaults to zero forque.

# Returns
- A tuple containing the resulting twist and pose vectors after the simulation step.

# Note
The function utilizes a differential equations solver (`DynamicalODEProblem` and `solve`) to compute the 
updated values of twist and pose based on the provided inputs and time step.
"""
function kinetic_step(
    twist::AbstractVector,
    pose::AbstractVector;
    inertia::MultiVector = pga_line(1,1,1,1,1,1),
    forque::Function = (args...; kwargs...) -> pga_line(0,0,0,0,0,0),
    axis_controls::Union{Vector, MVector} = MVector{0,Float64}(),
    button_controls::Union{Vector, MVector} = MVector{0,Float64}(),
    Δt::Float64 = 1.0,
)::Tuple{AbstractVector, AbstractVector}
    pose_motor::MultiVector = pga_motor(pose)
    pose_motor /= norm(pose_motor)
    pose = coefficients(pose_motor, PGA_MOTOR_INDICES_MVECTOR)

    p = (inertia=inertia, forque=forque, button_controls=button_controls, axis_controls=axis_controls)
    rbm_prob = DynamicalODEProblem(Δtwist!, Δpose!, twist, pose, (0.0, Δt), p)
    sol = solve(rbm_prob, Tsit5())

    return sol[end].x[1], sol[end].x[2]
end

"""
    function kinetic_step!(
        twist::AbstractVector,
        pose::AbstractVector,
        inertia::MultiVector,
        forque::Function,
        axis_controls::Vector,
        button_controls::Vector,
        Δt::Float64
    )::Nothing where {T<:Real}

Perform an in-place step of the rigid body motion simulation with forces and controls.

# Arguments
- `twist::AbstractVector`: Initial twist coefficients, representing velocity of the rigid body.
- `pose::AbstractVector`: Initial pose coefficients, representing the position and orientation.
- `inertia::MultiVector`: Moment of inertia of the rigid body, represented as a PGA Line.
- `forque::Function`: A function that returns the external combined force and torque as a PGA Line.
- `axis_controls::Vector`: List of continuous control values, e.g., joystick positions or rotary controls.
- `button_controls::Vector`: List of binary control values, e.g., button presses or switches.
- `Δt::Float64`: Time step for the simulation.

# Returns
- This function modifies the `twist` and `pose` arguments in-place and does not have a return value.

# Note
The function utilizes a differential equations solver (`DynamicalODEProblem` and `solve`) to compute the 
updated values of twist and pose based on the provided inputs and time step.
"""
function kinetic_step!(
    twist::AbstractVector,
    pose::AbstractVector;
    inertia::MultiVector = pga_line(1,1,1,1,1,1),
    forque::Function = (args...; kwargs...) -> pga_line(0,0,0,0,0,0),
    axis_controls::Union{Vector, MVector} = MVector{0,Float64}(),
    button_controls::Union{Vector, MVector} = MVector{0,Float64}(),
    Δt::Float64 = 1.0,
)::Nothing
    pose_motor::MultiVector = pga_motor(pose)
    pose_motor /= norm(pose_motor)
    pose .= coefficients(pose_motor, PGA_MOTOR_INDICES_MVECTOR)

    p = (inertia=inertia, forque=forque, button_controls=button_controls, axis_controls=axis_controls)
    rbm_prob = DynamicalODEProblem(Δtwist!, Δpose!, twist, pose, (0.0, Δt), p)
    sol = solve(rbm_prob, Tsit5())

    twist .= sol[end].x[1]
    pose .= sol[end].x[2]

    return nothing
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

```julia-repl
julia> twist = LinePGA(0.0, 0.0, 0.0, 0.1, 0.001, 0.0)
julia> pose = MotorPGA(-1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 3.0, -2.0)
julia> inertia = LinePGA(2, 1, 3, 1, 1, 1)
julia> step_count = 1000
julia> dt = 1.0
julia> euler_step(twist, pose, inertia, step_count, dt)
```
"""
function euler_step(
    twist::MultiVector,
    pose::MultiVector;
    step_count::Int64=1000000,
    inertia::MultiVector = pga_line(1,1,1,1,1,1),
    forque::Function = (args...; kwargs...) -> pga_line(0,0,0,0,0,0),
    axis_controls::Union{Vector, MVector} = MVector{0,Float64}(),
    button_controls::Union{Vector, MVector} = MVector{0,Float64}(),
    Δt::Float64 = 1.0,
)::Tuple{MultiVector, MultiVector}
    p = (inertia=inertia, forque=forque, button_controls=button_controls, axis_controls=axis_controls)

    for _ in 1:step_count
        twist += Δtwist(twist, pose, p, Δt) * Δt / step_count
        pose += Δpose(twist, pose) * Δt / step_count
    end
    pose /= norm(pose)

    return twist, pose
end

@component mutable struct Pose2D
    pose::MVector{4, Float64}
end

@component mutable struct Twist2D
    twist::MVector{3, Float64}
end

@component mutable struct Kinetics2D
    inertia::MultiVector
    forque::Function
end

struct KineticMover2D <: System end
function Overseer.update(::KineticMover2D, l::AbstractLedger, Δt::Float64)
    for e in @entities_in(l, Twist2D && Pose2D && Kinetics2D && AxisControls && ButtonControls)
        kinetic_step!(e.twist, e.pose; inertia=e.inertia, forque=e.forque, axis_controls=e.axes, button_controls=e.buttons, Δt=Δt)
    end
end

@component mutable struct Pose3D
    pose::MVector{8, Float64}
end

@component mutable struct Twist3D
    twist::MVector{6, Float64}
end

@component mutable struct Kinetics3D
    inertia::MultiVector
    forque::Function
end

struct KineticMover3D <: System end
function Overseer.update(::KineticMover3D, l::AbstractLedger, Δt::Float64)
    for e in @entities_in(l, Twist3D && Pose3D && Kinetics3D && AxisControls && ButtonControls)
        kinetic_step!(e.twist, e.pose; inertia=e.inertia, forque=e.forque, axis_controls=e.axes, button_controls=e.buttons, Δt=Δt)
    end
end
