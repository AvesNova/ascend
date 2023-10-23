using Test
include("../src/physics/physics.jl")


@testset "Physics Tests" begin
    @testset "Rigid Body Motion Tests" begin
        Δt = 100.0
        inertia_0 = pga_line(2, 1, 3, 1, 1, 1)

        @testset "Bi-Stable Rotation Test" begin
            twist_0 = pga_line(0.0, 0.0, 0.0, 0.1, 0.001, 0.0)
            pose_0 = pga_motor(-1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 3.0, -2.0)
            
            twist_1, pose_1 = euler_step(twist_0, pose_0; step_count=1000000, inertia=inertia_0, Δt=Δt)
            twist_e = coefficients(twist_1) |> collect
            pose_e = coefficients(pose_1) |> collect
            
            twist_T::MVector{6,Float64} = [0.0, 0.0, 0.0, 0.1, 0.001, 0.0]
            pose_T::MVector{8,Float64} = [-1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 3.0, -2.0]
            inertia_T::MVector{6,Float64} = [2, 1, 3, 1, 1, 1]
            
            twist_T, pose_T = kinetic_step(twist_T, pose_T; inertia=inertia_0, Δt=Δt)
            
            @test norm(twist_e - twist_T) < 1e-4
            @test norm(pose_e - pose_T) < 1e-4
        end

        @testset "Zero Dynamics Test" begin
            twist_0 = twist_t = MVector{6,Float64}([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            pose_0 = pose_t = MVector{8,Float64}([1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 3.0, -2.0])
            kinetic_step!(twist_t, pose_t; inertia=inertia_0, Δt=Δt)
            @test twist_t == twist_0
            @test pose_t == pose_0
        end

        @testset "Normalization Test" begin
            twist_0 = MVector{6,Float64}([0.0, 0.0, 0.0, 0.1, 0.001, 0.0])
            pose_0 = MVector{8,Float64}([1.0, 2.0, 2.0, 4.0, 5.0, 6.0, 7.0, 0.0])
            twist_t, pose_t = kinetic_step(twist_0, pose_0; inertia=inertia_0, Δt=Δt)
            pose_motor::MultiVector = pga_motor(pose_t)
            @test isapprox(norm(pose_motor), 1.0, atol=1e-4)
        end

        @testset "Symmetry Test" begin
            twist_0 = MVector{6,Float64}([0.0, 0.0, 0.0, 0.1, 0.001, 0.0])
            pose_0 = MVector{8,Float64}([1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 3.0, -2.0])

            twist_half, pose_half = kinetic_step(twist_0, pose_0; inertia=inertia_0, Δt=Δt/2)
            twist_full, pose_full = kinetic_step(twist_half, pose_half; inertia=inertia_0, Δt=Δt/2)
            
            twist_single, pose_single = kinetic_step(twist_0, pose_0; inertia=inertia_0, Δt=Δt)

            @test isapprox(twist_full, twist_single, atol=1e-4)
            @test isapprox(pose_full, pose_single, atol=1e-4)
        end

        @testset "Reversability Test" begin
            twist_0 = MVector{6,Float64}([0.0, 0.0, 0.0, 0.1, 0.001, 0.0])
            pose_0 = MVector{8,Float64}([1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 3.0, -2.0])
            
            twist_single, pose_single = kinetic_step(twist_0, pose_0; inertia=inertia_0, Δt=Δt)
            twist_reversed, pose_reversed = kinetic_step(twist_single, pose_single; inertia=inertia_0, Δt=-Δt)
            
            norm(twist_0 - twist_reversed) < 1e-4
            norm(pose_0 - pose_reversed) < 1e-4
        end
    end

    @testset "Rigid Body Motion Tests (In-place)" begin
        Δt = 100.0
        inertia_0::MultiVector = pga_line(2, 1, 3, 1, 1, 1)

        @testset "Bi-Stable Rotation Test" begin
            twist_0 = twist_t = MVector{6,Float64}([0.0, 0.0, 0.0, 0.1, 0.001, 0.0])
            pose_0 = pose_t = MVector{8,Float64}([-1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 3.0, -2.0])

            kinetic_step!(twist_t, pose_t; inertia=inertia_0, Δt=Δt)

            @test norm(collect(coefficients(pga_line(twist_t))) - collect(coefficients(pga_line(twist_0)))) < 1e-4
            @test norm(collect(coefficients(pga_motor(pose_t))) - collect(coefficients(pga_motor(pose_0)))) < 1e-4
        end

        @testset "Zero Dynamics Test" begin
            twist_0 = twist_t = MVector{6,Float64}([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            pose_0 = pose_t = MVector{8,Float64}([1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 3.0, -2.0])

            kinetic_step!(twist_t, pose_t; inertia=inertia_0, Δt=Δt)

            @test twist_t == twist_0
            @test pose_t == pose_0
        end

        @testset "Normalization Test" begin
            twist_0 = twist_t = MVector{6,Float64}([0.0, 0.0, 0.0, 0.1, 0.001, 0.0])
            pose_0 = pose_t = MVector{8,Float64}([1.0, 2.0, 2.0, 4.0, 5.0, 6.0, 7.0, 0.0])

            kinetic_step!(twist_t, pose_t; inertia=inertia_0, Δt=Δt)

            @test isapprox(norm(pga_motor(pose_t)), 1.0, atol=1e-4)
        end

        @testset "Symmetry Test" begin
            twist_0 = MVector{6,Float64}([0.0, 0.0, 0.0, 0.1, 0.001, 0.0])
            pose_0 = MVector{8,Float64}([1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 3.0, -2.0])

            twist_half, pose_half = kinetic_step(twist_0, pose_0; inertia=inertia_0, Δt=Δt/2)
            kinetic_step!(twist_half, pose_half; inertia=inertia_0, Δt=Δt/2)
            
            twist_single, pose_single = kinetic_step(twist_0, pose_0; inertia=inertia_0, Δt=Δt)

            @test isapprox(twist_half, twist_single, atol=1e-4)
            @test isapprox(pose_half, pose_single, atol=1e-4)
        end

        @testset "Reversability Test" begin
            twist_0 = MVector{6,Float64}([0.0, 0.0, 0.0, 0.1, 0.001, 0.0])
            pose_0 = MVector{8,Float64}([1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 3.0, -2.0])

            twist_t = copy(twist_0)
            pose_t = copy(pose_0)
            
            kinetic_step!(twist_t, pose_t; inertia=inertia_0, Δt=Δt)
            kinetic_step!(twist_t, pose_t; inertia=inertia_0, Δt=-Δt)
            
            @test isapprox(twist_t, twist_0, atol=1e-4)
            @test isapprox(pose_t, pose_0, atol=1e-4)
        end
    end

    @testset "Inertial Mapping Test" begin
        @testset "Inertia Map Inverse Test" begin
            twists = [
                pga_line(0.0, 0.0, 0.0, 0.1, 0.001, 0.0),
                pga_line(0.1, 0.2, 0.3, 0.1, 0.2, 0.3),
                pga_line(1.0, 1.0, 1.0, 1.0, 1.0, 1.0),
                pga_line(0.5, -0.5, 0.5, -0.5, 0.5, -0.5)
            ]
            
            inertias = [
                pga_line(2, 1, 3, 1, 1, 1),
                pga_line(1, 2, 1, 2, 1, 2),
                pga_line(0.5, 0.5, 0.5, 0.5, 0.5, 0.5),
                pga_line(3, 2, 1, 3, 2, 1)
            ]

            for (twist, inertia) in zip(twists, inertias)
                # Map twist to momentum using inertia_map
                momentum = inertia_map(twist, inertia)

                # Map momentum back to twist using inv_inertia_map
                twist_back = inv_inertia_map(momentum, inertia)

                # Adding test conditions
                @test norm(vector(twist - twist_back)) < 1e-15
            end
        end

        @testset "Error Propagation" begin
            twist = pga_line(0.1, 0.2, 0.3, 0.4, 0.5, 0.6)
            small_error = 1e-6
            twist_with_error = twist + pga_line(small_error, small_error, small_error, small_error, small_error, small_error)
            
            inertia = pga_line(1, 2, 3, 4, 5, 6)
            
            momentum1 = inertia_map(twist, inertia)
            momentum2 = inertia_map(twist_with_error, inertia)
            
            @test norm(vector(momentum1 - momentum2)) < 1e7
        end
    end
end
