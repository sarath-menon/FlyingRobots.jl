module quad_3d

using ModelingToolkit
using DiffEqCallbacks
using LinearAlgebra
using OrdinaryDiffEq
using Rotations
using NamedTupleTools
using StaticArrays
using ThreadPools

using GLMakie
using DataFrames
using CSV
using BenchmarkTools
using YAML
import Dates

GLMakie.activate!(inline=false)

using MtkLibrary
using FlyingRobots

include("mtk_models.jl")
include("types.jl")
include("logging.jl")

include("sim.jl")
include("modelling.jl")
include("vehicle.jl")

include("computer.jl")
include("controller_utilities.jl")
include("tasks.jl")


include("paths.jl")

include("integrator_callback.jl")

# build system model (Thread 2 )
@time system_build_task = @tspawnat 2 build_system_model()

# show visualizer (Thread 1)
plot_elements = FlyingRobots.Gui.show_visualizer()

# create computer 
flight_controller = create_computer("stm32")

#  get result from system build task 
sys, subsystems = fetch(system_build_task)

# Stepping simulation ----------------------------------------------------
# running vizulizer on 1st thread,(simulator+onboard computer) on 2nd thread
@time sim_task = @tspawnat 2 run_sim_stepping(sys, subsystems; save=false)
df = fetch(sim_task)

# plotting ----------------------------------------------------
FlyingRobots.Gui.set_sim_instance(plot_elements, df)

@async FlyingRobots.Gui.plot_position(plot_elements)
@async FlyingRobots.Gui.plot_orientation(plot_elements)
FlyingRobots.Gui.plot_control_input(plot_elements, motor_thrust_to_body_thrust)

flag = FlyingRobots.Gui.start_3d_animation(plot_elements)



end

