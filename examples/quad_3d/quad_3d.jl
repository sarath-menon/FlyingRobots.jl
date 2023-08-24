module quad_3d

using ModelingToolkit
using DiffEqCallbacks
using LinearAlgebra
using OrdinaryDiffEq
using Rotations
using NamedTupleTools
using StaticArrays

using GLMakie
using DataFrames
using CSV
using BenchmarkTools
using YAML

GLMakie.activate!(inline=false)

using MtkLibrary
using FlyingRobots

plot_elements, plot_data = FlyingRobots.Gui.show_visualizer()

include("dynamics_utilities.jl")
include("controller_utilities.jl")
include("types.jl")

include("dynamics.jl")
include("controller.jl")
include("plotting.jl")
include("logging.jl")
# include("gui.jl")

include("gui_utilities.jl")
include("scheduler.jl")
include("vehicle.jl")
include("sim.jl")
include("modelling.jl")

# read settings file 

vehicle_params_path = "/parameters/vehicle.yml"
ctrl_yaml_path = "/parameters/controller.yml"
sim_params_yaml = "/parameters/sim.yml"

vehicle_params = load_vehicle_params(vehicle_params_path)
ctrl_yaml = load_controller_params(ctrl_yaml_path)
sim_params = load_sim_params(sim_params_yaml, vehicle_params)

include("integrator_callback.jl")

# build system model
sys, subsystems = build_system_model()

## pid controllers
x_pos_pid = PID(ctrl_yaml[:position_controller][:pid_x])
y_pos_pid = PID(ctrl_yaml[:position_controller][:pid_y])
z_pos_pid = PID(ctrl_yaml[:position_controller][:pid_z])

roll_pid = PID(ctrl_yaml[:attitude_controller][:pid_roll])
pitch_pid = PID(ctrl_yaml[:attitude_controller][:pid_pitch])

@time sol = run_sim(sys, subsystems, sim_params, vehicle_params; save=true)

# plotting

# plot_position_attitude(sol)
state_indices = (position=(1, 3), orientation=(7, 10), motor_thrusts=(17, 21))

FlyingRobots.Gui.plot_reset(plot_data)

plot_position(plot_elements[:state_plots], plot_data, sol, state_indices)
# @time plot_orientation(Gui.state_plots, Gui.plot_data, sol, state_indices)

plot_control_input(plot_elements[:control_plots], plot_data, sol, state_indices, vehicle_params)

end