using FlyingRobots

using Test



let
    @testset "Quad2d" begin
        include("trajectory_generation.jl")
        include("dynamics.jl")
        include("controller.jl")
        include("typeutilities.jl")
        include("linearize.jl")
    end
end;