using Overseer
include("components.jl")

include("../input/input_manager.jl")
include("../physics/physics.jl")
include("../rendering/camera.jl")

struct PlayerController <: System end

function Overseer.update(::PlayerController, l::AbstractLedger)
    for e in @entities_in(l, ButtonControls, AxisControls, PlayerInputManager)
        get_player_input(e.button_controls, e.axis_controls, e.player_input_manager)
    end
end


struct AiController <: System end

function Overseer.update(::AiController, l::AbstractLedger)
    for e in @entities_in(l, ButtonControls, AxisControls, AiInputManager)
        get_ai_input(e)
    end
end


struct KinematicMover <: System end

function Overseer.update(::KinematicMover, l::AbstractLedger, Δt::Float64)
    for e in @entities_in(l, Twist && Pose && Kinetics)
        kinematic_step!(e.twist, e.pose, Δt)
    end
end


struct KineticMover <: System end

function Overseer.update(::KineticMover, l::AbstractLedger, Δt::Float64)
    for e in @entities_in(l, Twist && Pose && Kinetics && AxisControls && ButtonControls)
        kinetic_step!(e.twist, e.pose, e.inertia, e.forque, e.axis_controls, e.button_controls, Δt)
    end
end


struct CameraMover <: System end

function Overseer.update(::CameraMover, l::AbstractLedger)
    for e in @entities_in(l, Camera)

    end
end


struct Controller <: System end