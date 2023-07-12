using FlyingRobots

using Test

include("trajectory_generation.jl")
include("dynamics.jl")
include("typeutilities.jl")

let
    @testset "FlyingRobots.jl" begin
        TypeUtilitiesTest.run_tests()
        FlyingRobotsTest.run_tests()
        DynamicsTest.run_tests()
    end
end