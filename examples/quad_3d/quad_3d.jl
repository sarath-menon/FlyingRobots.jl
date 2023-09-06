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

import FlyingRobots.Simulation: SimAccMode, SimState, SimIdle, SimRunning
import FlyingRobots.Simulation: RealtimeSim, AcceleratedSim

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

# # show visualizer (Thread 1)
# plot_elements = FlyingRobots.Gui.show_visualizer()

# create computer 
flight_controller = create_computer("stm32")

# plotting ----------------------------------------------------
gui_setup_task = @tspawnat 1 FlyingRobots.Gui.show_visualizer()

#  get result from system build task 
sys, subsystems = fetch(system_build_task)
plot_elements = fetch(gui_setup_task)

## Testing
# df_empty = get_empty_df(sys, subsystems)
df_empty = get_empty_df(sys, subsystems)
sim_gui_ch = Channel{ODESolution}(10)

# # sim_cmd = Observable{Bool}(true)
# sim_cmd = Observable{SimState}()
# sim_acc_mode = Observable{SimAccMode}()

# sim_cmd[]
# sim_acc_mode[]

# c_buffer_len = 10
# c_buffer = CircularDeque{ODESolution}(c_buffer_len)
# plot_elements[:receiver_buffer] = c_buffer

# let
#     obs_func = on(sim_cmd, weak=true) do val
#         if sim_cmd[] == SimRunning()

#             if sim_acc_mode[] == RealtimeSim()
#                 Core.println("Starting Realtime simulation")

#                 # # clear existing plot data 
#                 # FlyingRobots.Gui.plot_reset(plot_elements)

#                 # start the realtime plotter
#                 gui_dynamic_plotter_task = @async FlyingRobots.Gui.gui_dynamic_plotter(plot_elements, sim_gui_ch, df_empty)

#                 # run sim on 2nd thread
#                 @time sim_task = @tspawnat 2 run_sim_stepping(sys, subsystems, sim_gui_ch, sim_cmd, sim_acc_mode; save=false)

#             elseif sim_acc_mode[] == AcceleratedSim()
#                 Core.println("Starting Accelerated simulation")

#                 # clear existing plot data 
#                 FlyingRobots.Gui.plot_reset(plot_elements)

#                 # run sim on 2nd thread
#                 @time sim_task = @tspawnat 2 run_sim_stepping(sys, subsystems, sim_gui_ch, sim_cmd, sim_acc_mode; save=false)
#                 df = fetch(sim_task)

#                 # plot the result
#                 FlyingRobots.Gui.set_sim_instance(plot_elements, df)
#                 FlyingRobots.Gui.plot_position(plot_elements)
#             end
#         end
#     end
# end

# obs_func = nothing

function start_realtime_sim(plot_elements, sim_gui_ch, df_empty)

    sim_cmd = plot_elements[:sim_cmd]
    sim_acc_mode = plot_elements[:sim_acc_mode]

    sim_gui_channel_lock = plot_elements[:locks][:sim_channel]
    gui_recv_buffer_lock = plot_elements[:locks][:recv_buffer]

    @async FlyingRobots.Gui.gui_receiver(plot_elements, sim_gui_ch, sim_gui_channel_lock)
    @async FlyingRobots.Gui.gui_dynamic_plotter(plot_elements, df_empty, gui_recv_buffer_lock)

    sim_task = @tspawnat 2 run_sim_stepping(sys, subsystems, sim_gui_ch, sim_cmd, sim_acc_mode; save=false)

    return sim_task
end

start_realtime_sim(plot_elements, sim_gui_ch, df_empty)


#Simulation ----------------------------------------------------
# running vizulizer on 1st thread,(simulator+onboard computer) on 2nd thread
# @time sim_task = @tspawnat 2 run_sim_stepping(sys, subsystems, sim_gui_ch, sim_cmd; save=false)
@time sim_task = @async run_sim_stepping(sys, subsystems, sim_gui_ch, sim_cmd, sim_acc_mode; save=false)
df = fetch(sim_task)

# plotting ----------------------------------------------------
plot_elements = FlyingRobots.Gui.show_visualizer()
FlyingRobots.Gui.plot_reset(plot_elements)

# connect observables
connect!(sim_cmd, plot_elements[:sim_cmd])
connect!(sim_acc_mode, plot_elements[:sim_acc_mode])

FlyingRobots.Gui.set_sim_instance(plot_elements, df)
# FlyingRobots.Gui.set_sim_flag(plot_elements, sim_cmd)


FlyingRobots.Gui.plot_position(plot_elements)
FlyingRobots.Gui.plot_orientation(plot_elements)
FlyingRobots.Gui.plot_control_input(plot_elements, motor_thrust_to_body_thrust)

@time sim_cmd = FlyingRobots.Gui.start_3d_animation(plot_elements)

end

