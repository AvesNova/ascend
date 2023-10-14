using CliffordAlgebras, StaticArrays

mutable struct Airfoil
    lift_slope
    skin_friction
    aoa_zero_lift
    stall_angle_low
    stall_angle_high
    chord
    span
    flap_fraction
    aspect_ratio
end

struct AeroCoefficients
    lift
    drag
    moment 
end

function lerp(a::Float64, b::Float64, t::Float64)
    return a + (b - a) * t
end

function flap_effectiveness_correction(flap_angle::Float64)
    normalized_value = (deg2rad(abs(flap_angle)) - 10) / 50
    return lerp(0.8, 0.4, normalized_value)
end

function calculate_coefficients(aoa, corrected_lift_slope, aoa_zero_lift, stall_angle_low, stall_angle_high)
    
end

function calculate_forces(airfoil::Airfoil, aircraft_twist::MultiVector, flight_surface_pose::MultiVector, flap_angle::Float64)
    (;lift_slope, skin_friction, chord, flap_fraction, aspect_ratio) = airfoil

    corrected_lift_slope = lift_slope * aspect_ratio / (aspect_ratio + 2 * (aspect_ratio + 4) / (aspect_ratio + 2))

    θ = acos(flap_fraction - 1)
    flap_effectiveness = 1 - (θ - sin(θ)) / π
    Δlift = corrected_lift_slope * flap_effectiveness * flap_effectiveness_correction(flap_angle) * flap_angle

    aoa_zero_lift_base = airfoil.aoa_zero_lift |> deg2rad
    aoa_zero_lift = aoa_zero_lift_base - Δlift / corrected_lift_slope

    stall_angle_low = airfoil.stall_angle_low |> deg2rad
    stall_angle_high = airfoil.stall_angle_high |> deg2rad

    flight_surface_twist = flight_surface_pose ≀ aircraft_twist
    dynamic_pressure = 0.5 * air_density * norm_sqr(flight_surface_twist)
    aoa = atan(flight_surface_twist.y, flight_surface_twist.z)

    aero_coefficients = calculate_coefficients(aoa, corrected_lift_slope, aoa_zero_lift, stall_angle_low, stall_angle_high)
end