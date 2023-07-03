using OrdinaryDiffEq

include("pga.jl")


function inertia_map(twist::MultiVector, inertia::MultiVector)::MultiVector
    return dual(twist) .* inertia
end

function inv_inertia_map(momentum::MultiVector, inertia::MultiVector)::MultiVector
    return dual(momentum ./ inertia)
end

function Δpose(twist::MultiVector, pose::MultiVector)::MultiVector
    return -0.5 * pose * twist
end

function Δpose!(Δpose::Vector, twist::Vector, pose::Vector, p, t::Real)
    twist_line = LinePGA(twist)
    pose_motor = MotorPGA(pose)
    Δpose_line = -0.5 * pose_motor * twist_line
    Δpose .= coefficients(Δpose_line, MOTOR_PGA_INDICES_VECTOR)
end

"""
    Δtwist(twist::MultiVector, inertia::MultiVector; forque::MultiVector = Line(0,0,0,0,0,0))::MultiVector

Acceleration = I⁻¹[Forque + Rate ×₋ I[Rate]]
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

function Δtwist!(Δtwist::Vector, twist::Vector, pose::Vector, p, t::Real)
    inertia = LinePGA(2, 1, 3, 1, 1, 1)
    twist_line = LinePGA(twist)

    Itwist::MultiVector = inertia_map(twist_line, inertia)
    momentum::MultiVector = twist_line ×₋ Itwist
    result_Δtwist = inv_inertia_map(momentum, inertia)

    Δtwist .= coefficients(result_Δtwist, LINE_PGA_INDICES_VECTOR)
end

function euler_step(twist::MultiVector, pose::MultiVector, inertia::MultiVector, step_count::Int, dt::Real)::Tuple{MultiVector, MultiVector}
    for _ in 1:step_count
        twist += Δtwist(twist, inertia) * dt / step_count
        pose += Δpose(twist, pose) * dt / step_count
    end
    pose /= norm(pose)

    return twist, pose
end

function init_rigid_body_motion(twist::Vector, pose::Vector, inertia::MultiVector; forque::Function)::SciMLBase.DynamicalODEProblem
    # TODO: add inertia and forque into p
    return DynamicalODEProblem(Δtwist!, Δpose!, twist, pose, tspan)
end

function step!(
    ode_problem::SciMLBase.DynamicalODEProblem,
    Δt::Real,
)   
    ode_problem.tspan = (0.0, Δt)
    sol = solve(ode_problem, Tsit5())

    ode_problem.twist = sol[end].x[1]

    pose = MotorPGA(sol[end]x.[2])
    pose /= norm(pose)
    ode_problem.pose .= coefficients(pose, MOTOR_PGA_INDICES_VECTOR)
end

twist_0::MultiVector = LinePGA(0.0, 0.0, 0.0, 0.1, 0.001, 0.0)
pose_0::MultiVector = MotorPGA(-1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 3.0, -2.0)
inertia_0::MultiVector = LinePGA(2, 1, 3, 1, 1, 1)

twist_1, pose_1 = euler_step(twist_0, pose_0, inertia_0, 1000000, 100)
twist_e = coefficients(twist_1) |> collect
pose_e = coefficients(pose_1) |> collect

twist_T::Vector{Float64} = [0.0, 0.0, 0.0, 0.1, 0.001, 0.0]
pose_T::Vector{Float64} = [-1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 3.0, -2.0]
inertia_T::Vector{Float64} = [2, 1, 3, 1, 1, 1]

u₀::Vector{Float64} = [pose_T; twist_T]

tspan = (0.0, 100.0)

prob = DynamicalODEProblem(Δtwist!, Δpose!, twist_T, pose_T, tspan)

sol = solve(prob, Tsit5())

print(sol[end])

twist_t = sol[end][1:6]
pose_t = sol[end][7:end]

@show norm(twist_e - twist_t)
@show norm(pose_e - pose_t)