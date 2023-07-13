

using FlyingRobots

using ForwardDiff


using StaticArrays

include("common.jl")
include("types.jl")
include("dynamics.jl")
include("control.jl")
include("linearize.jl")


export dynamics!, update_state!

selva() = println("selva")