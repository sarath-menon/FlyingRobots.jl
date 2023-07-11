module FlyingRobots
using Revise

# Use the README as the module docs
@doc let
    path = joinpath(dirname(@__DIR__), "README.md")
    include_dependency(path)
    read(path, String)
end FlyingRobots



include("dynamics/types.jl")
include("dynamics/utilities.jl")
include("dynamics/trajectory_generation.jl")
include("dynamics/quad_2d.jl")

Revise.track("src/dynamics/types.jl")
Revise.track("src/dynamics/trajectory_generation.jl")
Revise.track("src/dynamics/utilities.jl")
"""
    greet()

To greet new users
"""
greet() = "hello"

export greet

end
