module TrajectoryGeneration

using StaticArrays
using LinearAlgebra
using GeometryBasics
using Polynomials
using Distributions
using Rotations
using GLMakie

GLMakie.activate!(inline=false)

include("types.jl")
include("math.jl")
include("visualization.jl")
include("trajectory.jl")
include("obstacle.jl")


end # module TrajectoryGeneration
