module FlyingRobots

using NamedTupleTools

export FrRobotState, FrCtrlCmd, FrRobotDynamics, FrRobot
export FrCtrlCmd, FrActuatorCmd
export FrDigitalController
using MtkLibrary

using StaticArrays
using Rotations

import PrecompileTools

# Use the README as the module docs
@doc let
    path = joinpath(dirname(@__DIR__), "README.md")
    include_dependency(path)
    read(path, String)
end FlyingRobots

include("common/types.jl")
include("common/utilities/utilities.jl")

include("simulation/simulation.jl")
using .Simulation

include("gui/gui.jl")
using .Gui

include("control/control.jl")

include("computer/computer.jl")
include("dynamics/dynamics.jl")
include("models/models.jl")

using .Dynamics
using .Computer
using .Models

PrecompileTools.@setup_workload begin

    PrecompileTools.@compile_workload begin


        Gui.show_visualizer()

        Models.QuadcopterSystem()

    end
end

end
