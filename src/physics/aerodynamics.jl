using CliffordAlgebras, StaticArrays, Interact, Plots, Unitful

mutable struct Airfoil
    lift_slope::Float64
    skin_friction::Float64
    aoa_zero_lift::Float64
    stall_angle_low::typeof(1.0u"°")
    stall_angle_high::typeof(1.0u"°")
    chord::typeof(1.0u"m")
    area::typeof(1.0u"m^2")
    flap_fraction::Float64
    aspect_ratio::Float64
end

function create_airfoil(;
    lift_slope = 2π,
    skin_friction = 0.02,
    aoa_zero_lift = -2.0u"°",
    stall_angle_low = -15.0u"°",
    stall_angle_high = 15.0u"°",
    chord = 1.0u"m",
    area = 10.0u"m^2",
    flap_fraction = 0.2,
    aspect_ratio = 2.0,
)
    return Airfoil(lift_slope,
        skin_friction,
        aoa_zero_lift,
        stall_angle_low,
        stall_angle_high ,
        chord,
        area ,
        flap_fraction,
        aspect_ratio,
    )
end

@enum AeroCoefficients begin
    LIFT = 1
    DRAG
    MOMENT
end

function lerp(a, b, t)
    return a + (b - a) * t
end

function flap_effectiveness_correction(flap_angle)
    normalized_value = (rad2deg(abs(flap_angle)) - 10u"°") / 50u"°"
    return lerp(0.8, 0.4, normalized_value)
end

function moment_proportion(aoa_effective)
    return 0.25 - 0.175 * (1 - 2 * abs(aoa_effective) / π)
end

function calculate_coefficients_low_aoa(airfoil, aoa, corrected_lift_slope, aoa_zero_lift)
    lift_coefficient = corrected_lift_slope * (aoa - aoa_zero_lift)
    aoa_inducted = lift_coefficient / (π * airfoil.aspect_ratio)
    aoa_effective = aoa - aoa_zero_lift - aoa_inducted

    tangential_coefficient = airfoil.skin_friction * cos(aoa_effective)
    normal_coefficient = (lift_coefficient + sin(aoa_effective) * tangential_coefficient) / cos(aoa_effective)
    drag_coefficient = normal_coefficient * sin(aoa_effective) + tangential_coefficient * cos(aoa_effective)
    moment_coefficient = -normal_coefficient * moment_proportion(aoa_effective)

    return [lift_coefficient, drag_coefficient, moment_coefficient]
end

function friction_at_90_deg(flap_angle)
    return 1.98 - 4.26e-2 * flap_angle * flap_angle + 2.1e-1 * flap_angle
end

function calculate_coefficients_stall(airfoil, aoa, corrected_lift_slope, aoa_zero_lift, stall_angle_low, stall_angle_high, flap_angle)
    if aoa < stall_angle_low
        stall_angle = stall_angle_low
        lerp_param = (-π/2 - clamp(aoa, -π/2, π/2)) / (-π/2 - stall_angle)
    else
        stall_angle = stall_angle_high
        lerp_param = (π/2 - clamp(aoa, -π/2, π/2)) / (π/2 - stall_angle)
    end

    lift_coefficient_low_aoa = corrected_lift_slope * (stall_angle - aoa_zero_lift)
    aoa_induced = lift_coefficient_low_aoa / (π * airfoil.aspect_ratio)
    aoa_induced = lerp(0, aoa_induced, lerp_param)
    aoa_effective = aoa - aoa_zero_lift - aoa_induced

    normal_coefficient = friction_at_90_deg(flap_angle) * sin(aoa_effective) * (
        1 / (0.56 + 0.44 * abs(sin(aoa_effective))) - 0.41 * (1 - exp(-17 / airfoil.aspect_ratio))
    )
    tangential_coefficient = 0.5 * airfoil.skin_friction * cos(aoa_effective)

    lift_coefficient = normal_coefficient * cos(aoa_effective) - tangential_coefficient * sin(aoa_effective)
    drag_coefficient = normal_coefficient * sin(aoa_effective) + tangential_coefficient * cos(aoa_effective)
    moment_coefficient = -normal_coefficient * moment_proportion(aoa_effective)

    return [lift_coefficient, drag_coefficient, moment_coefficient]
end

function calculate_coefficients_transition_aoa(airfoil, aoa, corrected_lift_slope, aoa_zero_lift, stall_angle_low, stall_angle_low_padded, stall_angle_high, stall_angle_high_padded, flap_angle)
    if aoa > stall_angle_high
        stall_angle = stall_angle_high
        stall_angle_padded = stall_angle_high_padded
    else
        stall_angle = stall_angle_low
        stall_angle_padded = stall_angle_low_padded
    end

    aero_coefficients_low_aoa = calculate_coefficients_low_aoa(airfoil, stall_angle, corrected_lift_slope, aoa_zero_lift)
    aero_coefficients_stall = calculate_coefficients_stall(airfoil, stall_angle_padded, corrected_lift_slope, aoa_zero_lift, stall_angle_low, stall_angle_high, flap_angle)

    lerp_param = (aoa - stall_angle) / (stall_angle_padded - stall_angle)
    return lerp(aero_coefficients_low_aoa, aero_coefficients_stall, lerp_param)
end

function calculate_coefficients(airfoil, aoa, corrected_lift_slope, aoa_zero_lift, stall_angle_low, stall_angle_high, flap_angle)
    # low angle of attack
    if stall_angle_low <= aoa <= stall_angle_high
        return calculate_coefficients_low_aoa(airfoil, aoa, corrected_lift_slope, aoa_zero_lift)
    end

    stall_angle_low_padded = stall_angle_low - deg2rad(lerp(15, 5, (-rad2deg(flap_angle) + 50) / 100))
    stall_angle_high_padded = stall_angle_high + deg2rad(lerp(15, 5, (rad2deg(flap_angle) + 50) / 100))
    
    # medium angle of attack | lineararly interpolate low aoa and stall conditions
    if stall_angle_low_padded < aoa < stall_angle_high_padded
        return calculate_coefficients_transition_aoa(airfoil, aoa, corrected_lift_slope, aoa_zero_lift, stall_angle_low, stall_angle_low_padded, stall_angle_high, stall_angle_high_padded, flap_angle)
    end

    # high angle of attack | full stall
    return calculate_coefficients_stall(airfoil, aoa, corrected_lift_slope, aoa_zero_lift, stall_angle_low, stall_angle_high, flap_angle)
end

function get_corrected_lift_slope(lift_slope, aspect_ratio)
    return lift_slope * aspect_ratio / (aspect_ratio + 2 * (aspect_ratio + 4) / (aspect_ratio + 2))
end

function get_aoa_zero_lift(flap_fraction, flap_angle, aoa_zero_lift_deg)
    θ = acos(flap_fraction - 1)
    flap_effectiveness = 1 - (θ - sin(θ)) / π
    Δlift = corrected_lift_slope * flap_effectiveness * flap_effectiveness_correction(flap_angle) * flap_angle

    aoa_zero_lift_base = aoa_zero_lift_deg |> deg2rad
    aoa_zero_lift = aoa_zero_lift_base - Δlift / corrected_lift_slope

    return aoa_zero_lift
end

function calculate_forces(airfoil::Airfoil, aircraft_twist::MultiVector, flight_surface_pose::MultiVector, flap_angle::Float64, air_density::Float64)
    (;aspect_ratio) = airfoil
    corrected_lift_slope = airfoil.lift_slope * aspect_ratio / (aspect_ratio + 2 * (aspect_ratio + 4) / (aspect_ratio + 2))

    θ = acos(airfoil.flap_fraction - 1)
    flap_effectiveness = 1 - (θ - sin(θ)) / π
    Δlift = corrected_lift_slope * flap_effectiveness * flap_effectiveness_correction(flap_angle) * flap_angle

    aoa_zero_lift_base = airfoil.aoa_zero_lift |> deg2rad
    aoa_zero_lift = aoa_zero_lift_base - Δlift / corrected_lift_slope

    stall_angle_low = airfoil.stall_angle_low |> deg2rad
    stall_angle_high = airfoil.stall_angle_high |> deg2rad

    flight_surface_twist = flight_surface_pose ≀ aircraft_twist
    v_forward, v_up = flight_surface_twist.øz, flight_surface_twist.øy
    aoa = atan(v_forward, v_up)
    dynamic_pressure = 0.5 * air_density * norm_sqr(flight_surface_twist)

    aero_coefficients = calculate_coefficients(airfoil, aoa, corrected_lift_slope, aoa_zero_lift, stall_angle_low, stall_angle_high, flap_angle)

    lift_forque = flight_surface_pose ∧ basevector(CliffordAlgebra(:Klein), :y) * aero_coefficients[AeroCoefficients.LIFT |> Int] * dynamic_pressure * airfoil.area
    drag_forque = flight_surface_pose ∧ basevector(CliffordAlgebra(:Klein), :z) * aero_coefficients[AeroCoefficients.DRAG |> Int] * dynamic_pressure * airfoil.area
    moment_forque = flight_surface_pose ∧ basevector(CliffordAlgebra(:Klein), :yz) * aero_coefficients[AeroCoefficients.MOMENT |> Int] * dynamic_pressure * airfoil.area * airfoil.chord

    forque = lift_forque + drag_forque + moment_forque
    return forque
end

function test_coefficients(airfoil, aoa, flap_angle)
    (;aspect_ratio) = airfoil
    corrected_lift_slope = airfoil.lift_slope * aspect_ratio / (aspect_ratio + 2 * (aspect_ratio + 4) / (aspect_ratio + 2))

    θ = acos(airfoil.flap_fraction - 1)
    flap_effectiveness = 1 - (θ - sin(θ)) / π
    Δlift = corrected_lift_slope * flap_effectiveness * flap_effectiveness_correction(flap_angle) * flap_angle

    aoa_zero_lift_base = airfoil.aoa_zero_lift
    aoa_zero_lift = aoa_zero_lift_base - Δlift / corrected_lift_slope

    stall_angle_low = airfoil.stall_angle_low
    stall_angle_high = airfoil.stall_angle_high

    return calculate_coefficients(airfoil, aoa, corrected_lift_slope, aoa_zero_lift, stall_angle_low, stall_angle_high, flap_angle)
end

function plot_airfoil_coefficients(;kwargs...)
    airfoil = create_airfoil(;kwargs...)
    # AoA = [-pi:0.01:pi;]u"rad"
    AoA = [-50.0:50.0;]u"°"

    flap_angle = -0.0u"°"
    coefficients = hcat([test_coefficients(airfoil, aoa, flap_angle) for aoa in AoA]...)'

    lift = coefficients[:,1]
    drag = coefficients[:,2]
    moment = coefficients[:,3]
    # plot(AoA, coefficients)
    Plots.plot(AoA, lift; label="C_Lift")
    Plots.plot!(AoA, drag; label="C_Drag")
    Plots.plot!(AoA, moment; label="C_Moment")
end

if abspath(PROGRAM_FILE) == abspath(@__FILE__)
    plot_airfoil_coefficients(
        lift_slope=6.2,
        skin_friction=0.02,
        aoa_zero_lift=-2.0u"°",
        aspect_ratio=2.0,
    )
    print("aerodynamics.jl")
end
