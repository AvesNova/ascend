
abstract type FlyingObjectType end
abstract type PlaneType <: FlyingObjectType end
abstract type BulletType <: FlyingObjectType end

mutable struct Plane <: PlaneType
    position::Vector{Float32}
    color::Vector{Float32}
end

function Plane()
    return Plane(ones(4), ones(4))
end

mutable struct GameState
    players::Vector{GameObject}
    bullets::Union{Vector{GameObject}, Nothing}
end

function GameState()
    return GameState([GameObject()], [])
end

function update_game_state!(game_state)
    game_state.players[1].pose[1] = (cos(time()) + 1) / 2
end
