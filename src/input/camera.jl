using Overseer
using StaticArrays
using CliffordAlgebras
using Printf

include("input_manager.jl")

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
        d_x::Float64 = e.buttons[LEFT |> Int] - e.buttons[RIGHT |> Int]
        d_y::Float64 = e.buttons[DOWN |> Int] - e.buttons[UP |> Int]
        d_z::Float64 = e.buttons[BACKWARD |> Int] - e.buttons[FORWARD |> Int]
        
        translator_norm = max(norm([d_x, d_y, d_z]), 1.0)
        translator = pga_translator(100translator_norm, d_x, d_y, d_z)

        # Calculate rotation
        d_yz::Float64 = e.buttons[TURN_DOWN |> Int] - e.buttons[TURN_UP |> Int]
        d_zx::Float64 = e.buttons[TURN_LEFT |> Int] - e.buttons[TURN_RIGHT |> Int]
        d_xy::Float64 = e.buttons[TURN_CLOCKWISE |> Int] - e.buttons[TURN_ANTICLOCKWISE |> Int]

        rotor_norm = max(norm([d_yz, d_zx, d_xy]), 1.0)
        rotor = pga_rotor(100rotor_norm, d_yz, d_zx, d_xy)

        # Apply translation and rotation
        pga_pose = pga.motor.new(e.pose) * translator * rotor
        pga_pose /= norm(pga_pose)

        e.pose = coefficients(pga_pose, pga.motor.indices_mvector)

        # formatted_pose = join(["$(@sprintf("% 7.4f", val))" for val in e.pose], " ")
        # print("\r$formatted_pose")
    end
end

struct Pose2Buffer <: System end

function Overseer.update(::Pose2Buffer, l::AbstractLedger)
    for (i, e) in enumerate(@entities_in(l, Pose && ObjectBuffer))
        if i == 1
            e.object_buffer[8(i-1)+1 : 8i] = e.pose
        else
            e.object_buffer[8(i-1)+1 : 8i] = e.pose
            e.object_buffer[8(i-1)+2 : 8(i-1)+4] *= -1
            e.object_buffer[8(i-1)+6 : 8(i-1)+8] *= -1
        end
    end
end

