using FlyingRobots

using Test

include("trajectory_generation.jl")
include("dynamics.jl")

let
    @testset "FlyingRobots.jl" begin
        FlyingRobotsTest.run_tests()
        DynamicsTest.run_tests()
    end
end