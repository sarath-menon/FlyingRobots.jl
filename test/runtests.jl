using FlyingRobots

using Test

let

    @testset "FlyingRobots.jl" begin
        # # Write your tests here.
        include("trajectory_generation.jl")
        include("dynamics.jl")

    end
end