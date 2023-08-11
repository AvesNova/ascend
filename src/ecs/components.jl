using Overseer
using StaticArrays
using CliffordAlgebras

@component mutable struct Twist
    twist::MVector{6, Float64}
end

@component mutable struct Pose
    pose::MVector{8, Float64}
end

@component mutable struct Kinetics
    inertia::MultiVector
    forque::Function
end

@component mutable struct AeroControls
    pitch::Float64
    yaw::Float64
    roll::Float64
    flaps::Float64
    speedbrake::Float64
end

@component mutable struct LinearControls
    position::MVector{3, Float64}
    force::MVector{3, Float64}
end