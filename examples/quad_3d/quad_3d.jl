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
# using DataStructures

GLMakie.activate!(inline=false)

using MtkLibrary
using FlyingRobots

import FlyingRobots.Simulation: SimAccMode, SimState, SimIdle, SimRunning
import FlyingRobots.Simulation: RealtimeSim, AcceleratedSim
import FlyingRobots.Models: QuadcopterSystem
import FlyingRobots: update!

include("mtk_models.jl")
include("types.jl")
include("logging.jl")

include("sim.jl")
include("modelling.jl")
include("vehicle.jl")

include("computer.jl")
include("controller_utilities.jl")
include("tasks/tasks.jl")
include("paths.jl")
include("integrator_callback.jl")
include("gui_helper.jl")
include("joystick.jl")


import .Joystick

params_dict = Dict{Symbol,Union{NamedTuple,Dict}}()
params_task = @tspawnat 2 @async params_auto_updater(params_dict)

# to load params manually for initialization
load_params!(params_dict)

# build system model (Thread 2 )
system_build_task = @tspawnat 2 QuadcopterSystem()

# show visualizer (Thread 1)
plot_elements = FlyingRobots.Gui.show_visualizer()

# get output from  async tasks
sys, subsystems = fetch(system_build_task)

# create computer 
flight_controller = create_computer("stm32", params_dict)

# @code_warntype create_computer("stm32", folder_path * vehicle_params_path)

# gui, sim setup ----------------------------------------------------
df_empty = get_empty_df(sys, subsystems)
sim_gui_ch = Channel{ODESolution}(10)

# don't run more than once
obs_func = define_gui_sim_interactions(plot_elements, sim_gui_ch, df_empty)

# Manual testing ----------------------------------------------------
start_realtime_sim(plot_elements, sim_gui_ch, df_empty)
start_accelerated_sim(plot_elements, sim_gui_ch)

# Joystick
js = Joystick.connect_joystick()
js_state = Joystick.get_joystick_state(js)

position_controller(P_PosController(), flight_controller, 20)
velocity_controller(Pid_VelController(), flight_controller, 20)
attitude_controller(SimplePid_AttitudeController(), flight_controller, 20)


FlyingRobots.reset!(flight_controller, params_dict)

flight_controller.ram_memory[:task_mem][:position_controller]


# plotting ----------------------------------------------------
FlyingRobots.Gui.plot_reset(plot_elements)

FlyingRobots.Gui.open_second_window()

# # connect observables
# connect!(sim_cmd, plot_elements[:sim_cmd])
# connect!(sim_acc_mode, plot_elements[:sim_acc_mode])

# FlyingRobots.Gui.set_sim_instance(plot_elements, df)
# FlyingRobots.Gui.set_sim_flag(plot_elements, sim_cmd)


# FlyingRobots.Gui.plot_position(plot_elements)
# FlyingRobots.Gui.plot_orientation(plot_elements)
# FlyingRobots.Gui.plot_control_input(plot_elements, motor_thrust_to_body_thrust)

@time sim_cmd = FlyingRobots.Gui.start_3d_animation(plot_elements)

end

