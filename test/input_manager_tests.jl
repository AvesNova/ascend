using Test, DataStructures
include("../src/input/input_manager.jl")


@testset "Input Manager Tests" begin
    @testset "Zero Action Vector" begin
        expected = MVector{ActionCount}(zeros(ActionCount))
        actual = zero_action_vector()
        @test expected == actual
    end

    @testset "Get Button Map With Dictionary" begin
        input_map = OrderedDict("KEY_W" => "FORWARD", "KEY_S" => "BACKWARD", "KEY_A" => "LEFT", "KEY_D" => "RIGHT")
        actual = get_button_map(input_map)
        expected = SMatrix{4,2,Int}([GLFW.KEY_W FORWARD; GLFW.KEY_S BACKWARD; GLFW.KEY_A LEFT; GLFW.KEY_D RIGHT] .|> Int)
        @test expected == actual
    end

    @testset "Get Button Map With Nothing" begin
        actual = get_button_map(nothing)
        expected = SMatrix{0,2}([])
        @test expected == actual
    end
end
