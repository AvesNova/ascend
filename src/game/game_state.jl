include("../physics/pga.jl")
include("../physics/physics.jl")

abstract type Entity end

mutable struct PhysicsComponent
    twist::MultiVector
    pose::MultiVector
    inertia::MultiVector
    forque::Function
end

mutable struct Player <: Entity
    physics::PhysicsComponent
end

function Player()
    twist::MultiVector = LinePGA(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    pose::MultiVector = MotorPGA(1.0, 0.0, 0.0, 0.0, 0.0, 1.0, -3.0, 0.0)
    inertia::MultiVector = LinePGA(1, 1, 1, 1, 1, 1)
    forque(twist, pose, t) = LinePGA(0,0,0,0,0,0)

    return Cube(
        PhysicsComponent(twist, pose, inertia, forque),
    )
end

mutable struct Cube <: Entity
    physics::PhysicsComponent
end

function Cube()
    twist::MultiVector = LinePGA(0.0, 0.0, 0.0, 0.1, 0.001, 0.0)
    pose::MultiVector = MotorPGA(1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 3.0, -2.0)
    inertia::MultiVector = LinePGA(1, 1, 1, 2, 1, 3)
    forque(twist, pose, t) = LinePGA(0,0,0,0,0,0)

    return Cube(
        PhysicsComponent(twist, pose, inertia, forque),
    )
end

mutable struct Environment
    Δt <: Real
end

mutable struct GameState
    environment::Environment
    entities::Dict{Symbol, Entity}
end

function GameState()
    player = Player()
    cube = Cube()
    return GameState(
        Environment(1/30),
        Dict(
            :player => player,
            :cube => cube,
        )
    )
end