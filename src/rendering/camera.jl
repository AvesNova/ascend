using Overseer
using StaticArrays
using CliffordAlgebras

include("../input/input_manager.jl")
include("../physics/physics.jl")

@component mutable struct Camera
    fov::Float64
end

function Camera()
    fov = 90.0
    return Camera(fov)
end

struct CameraMover <: System end

function Overseer.update(::CameraMover, l::AbstractLedger)
    for e in @entities_in(l, Actions && Camera && Pose)
        # Calculate translation
        d_x::Float64 = e.buttons[RIGHT |> Int] - e.buttons[LEFT |> Int]
        d_y::Float64 = e.buttons[FORWARD |> Int] - e.buttons[BACKWARD |> Int]
        d_z::Float64 = 0.0
        
        translator_norm = max(norm([d_x, d_y, d_z]), 1.0)
        translator = pga_translator(1000translator_norm, d_x, d_y, d_z)

        # Calculate rotation
        d_yz::Float64 = 0.0
        d_zx::Float64 = 0.0
        d_xy::Float64 = e.buttons[TURNRIGHT |> Int] - e.buttons[TURNLEFT |> Int]

        rotor_norm = max(norm([d_yz, d_zx, d_xy]), 1.0)
        rotor = pga_rotor(100rotor_norm, d_yz, d_zx, d_xy)

        # Apply translation and rotation
        pga_pose = pga_motor(e.pose) * translator * rotor
        pga_pose /= norm(pga_pose)

        e.pose = coefficients(pga_pose, PGA_MOTOR_INDICES_MVECTOR)
        # print("\r$(round.(e.pose; digits=4))")
    end
end

struct Pose2Buffer <: System end

function Overseer.update(::Pose2Buffer, l::AbstractLedger)
    for e in @entities_in(l, Pose && ObjectBuffer)
        print("\r$(round.(e.pose; digits=4))  $(round.(e.object_buffer; digits=4))")
        e.object_buffer[14:16] = e.pose[6:8]
    end
end
