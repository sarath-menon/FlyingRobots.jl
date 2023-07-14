module FlyingRobots

using Revise
using NamedTupleTools

export FrRobotState, FrCtrlCmd, FrRobotDynamics, FrRobot
export FrCtrlCmd, FrActuatorCmd
export FrDigitalController


# Use the README as the module docs
@doc let
    path = joinpath(dirname(@__DIR__), "README.md")
    include_dependency(path)
    read(path, String)
end FlyingRobots


include("constants.jl")
include("types.jl")
include("type_utilities.jl")

include("logger_utilities.jl")

include("dynamics/math_helper.jl")

include("dynamics/trajectory_generation.jl")
include("dynamics/plotting.jl")
include("dynamics/linearize.jl")
include("dynamics/sim.jl")


include("control/lqr.jl")

include("./../examples/quad_2d/types.jl")

include("dynamics/sim_new.jl")


# Revise.track("src/dynamics/types.jl")
# Revise.track("src/dynamics/trajectory_generation.jl")
# Revise.track("src/dynamics/utilities.jl")
# Revise.track("src/dynamics/plotting.jl")

# Revise.track("src/dynamics/linearize.jl")
# Revise.track("src/dynamics/sim.jl")
# Revise.track("src/dynamics/math_helper.jl")

# Revise.track("src/control/lqr.jl")

"""
    greet()

To greet new users
"""
greet() = "hello"

export greet

end
