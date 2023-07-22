using Test

include("../src/physics/physics.jl")


@testset "Physics Tests" begin
    @testset "Rigid Body Motion Tests" begin
        Δt = 100.0
        inertia_0::MultiVector = LinePGA(2, 1, 3, 1, 1, 1)

        @testset "Bi-Stable Rotation Test" begin
            twist_0::MultiVector = LinePGA(0.0, 0.0, 0.0, 0.1, 0.001, 0.0)
            pose_0::MultiVector = MotorPGA(-1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 3.0, -2.0)
            
            twist_1, pose_1 = euler_step(twist_0, pose_0, inertia_0, 1000000, 100)
            twist_e = coefficients(twist_1) |> collect
            pose_e = coefficients(pose_1) |> collect
            
            twist_T::Vector{Float64} = [0.0, 0.0, 0.0, 0.1, 0.001, 0.0]
            pose_T::Vector{Float64} = [-1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 3.0, -2.0]
            inertia_T::Vector{Float64} = [2, 1, 3, 1, 1, 1]
            
            twist_t, pose_t = rbm_physics_step(twist_T, pose_T, inertia_0, Δt)
            
            @test norm(twist_e - twist_t) < 1e-4
            @test norm(pose_e - pose_t) < 1e-4
        end

        @testset "Zero Dynamics Test" begin
            twist_0 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            pose_0 = [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 3.0, -2.0]
            twist_t, pose_t = rbm_physics_step(twist_0, pose_0, inertia_0, Δt)
            @test twist_t == twist_0
            @test pose_t == pose_0
        end

        @testset "Zero Dynamics MulitVector Test" begin
            twist_0 = LinePGA(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            pose_0 = MotorPGA(1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 3.0, -2.0)
            twist_t, pose_t = rbm_physics_step(twist_0, pose_0, inertia_0, Δt)
            @test twist_t == twist_0
            @test pose_t == pose_0
        end

        @testset "Normalization Test" begin
            twist_0 = [0.0, 0.0, 0.0, 0.1, 0.001, 0.0]
            pose_0 = [1.0, 2.0, 2.0, 4.0, 5.0, 6.0, 7.0, 8.0]
            _, pose_t = rbm_physics_step(twist_0, pose_0, inertia_0, Δt)
            pose_motor::MultiVector = MotorPGA(pose_t)
            @test isapprox(norm(pose_motor), 1.0, atol=1e-4)
        end

        @testset "Normalization MulitVector Test" begin
            twist_0 = LinePGA(0.0, 0.0, 0.0, 0.1, 0.001, 0.0)
            pose_0 = MotorPGA(1.0, 2.0, 2.0, 4.0, 5.0, 6.0, 7.0, 8.0)
            _, pose_t = rbm_physics_step(twist_0, pose_0, inertia_0, Δt)
            
            @test isapprox(norm(pose_t), 1.0, atol=1e-4)
        end

        @testset "Symmetry Test" begin
            twist_0 = [0.0, 0.0, 0.0, 0.1, 0.001, 0.0]
            pose_0 = [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 3.0, -2.0]

            twist_half, pose_half = rbm_physics_step(twist_0, pose_0, inertia_0, Δt/2)
            twist_full, pose_full = rbm_physics_step(twist_half, pose_half, inertia_0, Δt/2)
            
            twist_single, pose_single = rbm_physics_step(twist_0, pose_0, inertia_0, Δt)

            @test isapprox(twist_full, twist_single, atol=1e-4)
            @test isapprox(pose_full, pose_single, atol=1e-4)
        end

        @testset "Symmetry MulitVector Test" begin
            twist_0 = LinePGA(0.0, 0.0, 0.0, 0.1, 0.001, 0.0)
            pose_0 = MotorPGA(1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 3.0, -2.0)

            twist_half, pose_half = rbm_physics_step(twist_0, pose_0, inertia_0, Δt/2)
            twist_full, pose_full = rbm_physics_step(twist_half, pose_half, inertia_0, Δt/2)
            
            twist_single, pose_single = rbm_physics_step(twist_0, pose_0, inertia_0, Δt)

            @test norm(vector(twist_full - twist_single)) < 1e-4
            @test norm(vector(pose_full - pose_single)) < 1e-4
        end

        @testset "Reversability Test" begin
            twist_0 = [0.0, 0.0, 0.0, 0.1, 0.001, 0.0]
            pose_0 = [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 3.0, -2.0]
            
            twist_single, pose_single = rbm_physics_step(twist_0, pose_0, inertia_0, Δt)
            twist_reversed, pose_reversed = rbm_physics_step(twist_single, pose_single, inertia_0, -Δt)
            
            norm(twist_0 - twist_reversed) < 1e-4
            norm(pose_0 - pose_reversed) < 1e-4
        end

        @testset "Reversability MulitVector Test" begin
            twist_0 = LinePGA(0.0, 0.0, 0.0, 0.1, 0.001, 0.0)
            pose_0 = MotorPGA(1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 3.0, -2.0)
            
            twist_single, pose_single = rbm_physics_step(twist_0, pose_0, inertia_0, Δt)
            twist_reversed, pose_reversed = rbm_physics_step(twist_single, pose_single, inertia_0, -Δt)
            
            norm(vector(twist_0 - twist_reversed)) < 1e-4
            norm(vector(pose_0 - pose_reversed)) < 1e-4
        end
    end

    @testset "Inertial Mapping Test" begin
        @testset "Inertia Map Inverse Test" begin
            twists = [
                LinePGA(0.0, 0.0, 0.0, 0.1, 0.001, 0.0),
                LinePGA(0.1, 0.2, 0.3, 0.1, 0.2, 0.3),
                LinePGA(1.0, 1.0, 1.0, 1.0, 1.0, 1.0),
                LinePGA(0.5, -0.5, 0.5, -0.5, 0.5, -0.5)
            ]
            
            inertias = [
                LinePGA(2, 1, 3, 1, 1, 1),
                LinePGA(1, 2, 1, 2, 1, 2),
                LinePGA(0.5, 0.5, 0.5, 0.5, 0.5, 0.5),
                LinePGA(3, 2, 1, 3, 2, 1)
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
            twist = LinePGA(0.1, 0.2, 0.3, 0.4, 0.5, 0.6)
            small_error = 1e-6
            twist_with_error = twist + LinePGA(small_error, small_error, small_error, small_error, small_error, small_error)
            
            inertia = LinePGA(1, 2, 3, 4, 5, 6)
            
            momentum1 = inertia_map(twist, inertia)
            momentum2 = inertia_map(twist_with_error, inertia)
            
            @test norm(vector(momentum1 - momentum2)) < 1e7
        end
    end
end
