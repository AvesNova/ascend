include("../physics/physics.jl")

# abstract type Entity end

# mutable struct Player <: Entity
#     physics::PhysicsComponent
# end

# function Player()
#     twist::MVector{6,Float64} = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
#     pose::MVector{8,Float64} = [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, -3.0, 0.0]
#     inertia::MultiVector = LinePGA(1, 1, 1, 1, 1, 1)
#     forque(twist, pose, t) = LinePGA(0,0,0,0,0,0)

#     return Cube(
#         PhysicsComponent(twist, pose, inertia, forque),
#     )
# end

# mutable struct Cube <: Entity
#     physics::PhysicsComponent
# end

# function Cube()
#     twist::MVector{6,Float64} = [0.0, 0.0, 0.0, 0.1, 0.001, 0.0]
#     pose::MVector{8,Float64} = [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 3.0, -2.0]
#     inertia::MultiVector = LinePGA(1, 1, 1, 2, 1, 3)
#     forque(twist, pose, t) = LinePGA(0,0,0,0,0,0)

#     return Cube(
#         PhysicsComponent(twist, pose, inertia, forque),
#     )
# end

# mutable struct Environment{T <: Real}
#     t_last::T
#     Δt_target::T
# end

# function Environment(Δt_target::T) where {T <: Real}
#     return Environment(time(), Δt_target)
# end

# mutable struct GameState
#     environment::Environment
#     # entities::Dict{Symbol, Entity}
# end

# function GameState()
#     return GameState(
#         Environment(1/30),
#         # Dict(
#         #     :player => player,
#         #     :cube => cube,
#         # )
#     )
# end

# function update_game_state!(game_state::GameState, input_manager::InputManager)
#     t_current = time()
#     Δt = t_current - game_state.environment.t_last
#     game_state.environment.t_last = t_current

#     for entity in values(game_state.entities)
#         rbm_physics_step!(entity.physics, Δt)
#     end
# end
