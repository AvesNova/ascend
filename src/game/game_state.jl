include("../physics/pga.jl")
include("../physics/physics.jl")

mutable struct FlyingObject
    pose::MultiVector
    twist::MultiVector
    inertia::MultiVector
end

function FlyingObject()
    return FlyingObject(
        Motor(),  # Origin
        LinePGA(),  # No movement
        LinePGA(1.0, 1.0, 1.0, 0.4, 0.4, 0.4),  # Inertia of sphere with length and mass 1
    )
end

mutable struct GameState
    players::Vector{FlyingObject}
    bullets::Union{Vector{FlyingObject}, Nothing}
    environment
end

function GameState()
    return GameState([FlyingObject()], [], 1)
end

function test_movement(pose::MultiVector, actions::Action, axes::Axis)
    
end

function update_game_state!(game_state::GameState, input_state::InputManager)
    game_state.players[1].pose[1] = (cos(time()) + 1) / 2
end

"""
    get_player_poses(game_state::GameState)::Vector{Float32}

Return a flat array of the pose coefficients of all players in the game state. Each pose's 
coefficients are converted to `Float32` and added to the array in order.

# Examples
```jldoctest
julia> game_state = GameState(
    [FlyingObject(Motor(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0), [0.1f0, 0.2f0, 0.3f0, 0.4f0], 10, 100.0), 
    FlyingObject(Motor(1.1, 2.2, 3.3, 4.4, 5.5, 6.6, 7.7, 8.8), [0.5f0, 0.6f0, 0.7f0, 0.8f0], 10, 100.0)], 
    nothing)

julia> get_player_poses(game_state)
16-element Vector{Float32}:
 1.0
 2.0
 3.0
 4.0
 5.0
 6.0
 7.0
 8.0
 1.1
 2.2
 3.3
 4.4
 5.5
 6.6
 7.7
 8.8
```
"""
function get_player_poses(game_state::GameState)::Vector{Float32}
    return [Float32(c) for player in game_state.players for c in coefficients(player.pose)]
end

"""
    get_player_render_info(game_state::GameState)::Vector{Float32}

Return a flat array of the pose coefficients and color elements of all players in the game state. 
Each pose's coefficients and color elements are converted to `Float32` and added to the array in order.

# Examples
```jldoctest
julia> game_state = GameState(
    [FlyingObject(Motor(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0), [0.1f0, 0.2f0, 0.3f0, 0.4f0], 10, 100.0), 
    FlyingObject(Motor(1.1, 2.2, 3.3, 4.4, 5.5, 6.6, 7.7, 8.8), [0.5f0, 0.6f0, 0.7f0, 0.8f0], 10, 100.0)], 
    nothing)

julia> get_player_render_info(game_state)
24-element Vector{Float32}:
 1.0
 2.0
 3.0
 4.0
 5.0
 6.0
 7.0
 8.0
 0.1
 0.2
 0.3
 0.4
 1.1
 2.2
 3.3
 4.4
 5.5
 6.6
 7.7
 8.8
 0.5
 0.6
 0.7
 0.8
````
"""
function get_player_render_info(game_state::GameState)::Vector{Float32}
    return [f for player in game_state.players for f in vcat([Float32(c) for c in coefficients(player.pose)], player.color)]
end


