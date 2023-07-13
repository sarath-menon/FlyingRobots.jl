using FlyingRobots

using Test

include("trajectory_generation.jl")
include("dynamics.jl")
include("controller.jl")
include("typeutilities.jl")
include("linearize.jl")

let
    @testset "FlyingRobots.jl" begin
        TypeUtilitiesTest.run_tests()
        FlyingRobotsTest.run_tests()
        DynamicsTest.run_tests()
        ControllerTest.run_tests()
    end
end;