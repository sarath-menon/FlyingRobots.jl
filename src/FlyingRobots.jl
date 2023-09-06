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

using .Dynamics
using .Computer

PrecompileTools.@setup_workload begin

    PrecompileTools.@compile_workload begin

        # show visualizer (Thread 1)
        Gui.show_visualizer()

    end
end

end
