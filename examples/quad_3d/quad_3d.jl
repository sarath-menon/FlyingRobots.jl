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
using DataStructures

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
include("gui_tasks.jl")

include("paths.jl")

include("integrator_callback.jl")

# build system model (Thread 2 )
@time system_build_task = @tspawnat 2 build_system_model()

# # show visualizer (Thread 1)
# plot_elements = FlyingRobots.Gui.show_visualizer()

# create computer 
flight_controller = create_computer("stm32")

#  get result from system build task 
sys, subsystems = fetch(system_build_task)

## Testing

c1 = Channel{ODESolution}(10)
# df_empty = get_empty_df(sys, subsystems)

flag = Observable{Bool}(true)

gui_dynamic_plotter_task = @async gui_dynamic_plotter(flag, c1, plot_elements, df_empty)

#Simulation ----------------------------------------------------
# running vizulizer on 1st thread,(simulator+onboard computer) on 2nd thread
@time sim_task = @tspawnat 2 run_sim_stepping(sys, subsystems, c1, flag; save=false)
#@time sim_task = @async run_sim_stepping(sys, subsystems, c1, flag; save=false)
df = fetch(sim_task)


flag[] = false

# plotting ----------------------------------------------------
plot_elements = FlyingRobots.Gui.show_visualizer()
FlyingRobots.Gui.plot_reset(plot_elements)

FlyingRobots.Gui.set_sim_instance(plot_elements, df)

FlyingRobots.Gui.plot_position(plot_elements)
FlyingRobots.Gui.plot_orientation(plot_elements)
FlyingRobots.Gui.plot_control_input(plot_elements, motor_thrust_to_body_thrust)

@time flag = FlyingRobots.Gui.start_3d_animation(plot_elements)

reference_generator(Main.quad_3d.Circle_TrajectoryGen(), flight_controller, 10)

scheduler(flight_controller)

Main.quad_3d.Circle_TrajectoryGen == Circle_TrajectoryGen

end

