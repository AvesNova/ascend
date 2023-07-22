abstract type GameObject end
abstract type PhxObject <: GameObject end

mutable struct Aircraft <: GameObject
    pose::MultiVector
    velocity::MultiVector
    inertia::MultiVector
    fuel::Real
    color::Vector{Float32}
end

mutable struct Player <: Aircraft
    ammo::Int
end

mutable struct Bullet <: GameObject
    pose::MultiVector
    velocity::MultiVector
    color::Vector{Float32}
end

mutable struct Environment
    # Specify your environment properties here
end

mutable struct GameState
    players::Vector{Player}
    bullets::Vector{Bullet}
    environment::Environment
end
