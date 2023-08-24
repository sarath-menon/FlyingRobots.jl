module FlyingRobots

using NamedTupleTools

export FrRobotState, FrCtrlCmd, FrRobotDynamics, FrRobot
export FrCtrlCmd, FrActuatorCmd
export FrDigitalController
using MtkLibrary

using StaticArrays
using Rotations

# Use the README as the module docs
@doc let
    path = joinpath(dirname(@__DIR__), "README.md")
    include_dependency(path)
    read(path, String)
end FlyingRobots

include("types.jl")

include("lib/control/control.jl")
include("gui.jl")

include("lib/utilities/utilities.jl")

end
