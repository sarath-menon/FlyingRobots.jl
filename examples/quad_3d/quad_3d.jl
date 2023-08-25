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
import Dates

GLMakie.activate!(inline=false)

using MtkLibrary
using FlyingRobots

plot_elements = FlyingRobots.Gui.show_visualizer()

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
# read settings file 
folder_path = pwd() * "/examples/quad_3d"

vehicle_params_path = "/parameters/vehicle.yml"
ctrl_yaml_path = "/parameters/controller.yml"
sim_params_path = "/parameters/sim.yml"

vehicle_params = load_vehicle_params(folder_path * vehicle_params_path)
ctrl_yaml = load_controller_params(folder_path * ctrl_yaml_path)
sim_params = load_sim_params(folder_path * sim_params_path, vehicle_params)

include("integrator_callback.jl")

# build system model
sys, subsystems = build_system_model()

## pid controllers
x_pos_pid = PID(ctrl_yaml[:position_controller][:pid_x])
y_pos_pid = PID(ctrl_yaml[:position_controller][:pid_y])
z_pos_pid = PID(ctrl_yaml[:position_controller][:pid_z])

roll_pid = PID(ctrl_yaml[:attitude_controller][:pid_roll])
pitch_pid = PID(ctrl_yaml[:attitude_controller][:pid_pitch])

@time sol, df = run_sim(sys, subsystems, sim_params, vehicle_params; save=true)

# plotting

FlyingRobots.Gui.plot_reset(plot_data)

plot_position(plot_elements[:state_plots], plot_data, df)
plot_orientation(plot_elements[:state_plots], plot_data, df)

plot_control_input(plot_elements[:control_plots], plot_data, df, vehicle_params)

flag = FlyingRobots.Gui.plot_3d_trajectory(df, plot_elements)

## new gui
using FlyingRobots

plot_elements = FlyingRobots.Gui.show_visualizer()

FlyingRobots.Gui.set_sim_instance(plot_elements, df)

#flag = FlyingRobots.Gui.start_3d_animation(plot_elements)

FlyingRobots.Gui.plot_position(plot_elements)
FlyingRobots.Gui.plot_orientation(plot_elements)
FlyingRobots.Gui.plot_control_input(plot_elements, motor_thrust_to_body_thrust)

plot_elements[:visualizer_3d]
## to replay log file
# # load parameters
# log_file_path = pwd() * "/logs/Fri,25-Aug-2023 16-01-45"

# vehicle_params_path = "/parameters/vehicle.yml"
# ctrl_yaml_path = "/parameters/controller.yml"
# sim_params_path = "/parameters/sim.yml"

# vehicle_params = load_vehicle_params(log_file_path * vehicle_params_path)
# ctrl_yaml = load_controller_params(log_file_path * ctrl_yaml_path)
# sim_params = load_sim_params(log_file_path * sim_params_path, vehicle_params)

# # load data
# log_sim_output = CSV.read(log_file_path * "/sim_output.csv", DataFrame)

# plot_position(plot_elements[:state_plots], plot_data, log_sim_output)
# plot_orientation(plot_elements[:state_plots], plot_data, log_sim_output)

# plot_control_input(plot_elements[:control_plots], plot_data, log_sim_output, vehicle_params)

end