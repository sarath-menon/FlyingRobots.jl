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

include("dynamics_utilities.jl")
include("mtk_models.jl")
include("types.jl")

include("dynamics.jl")
include("logging.jl")

include("sim.jl")
include("modelling.jl")
include("vehicle.jl")
include("computer.jl")

# show visualizer
plot_elements = FlyingRobots.Gui.show_visualizer()

# read settings file 
folder_path = pwd() * "/examples/quad_3d"

vehicle_params_path = "/parameters/vehicle.yml"

sim_params_path = "/parameters/sim.yml"
ctrl_yaml_path = "/parameters/controller.yml"

vehicle_yaml = YAML.load_file(folder_path * vehicle_params_path; dicttype=Dict{Symbol,Any})
recursive_dict_to_namedtuple(vehicle_yaml[:computer])

# build system model
sys, subsystems = build_system_model()

vehicle_params = load_vehicle_params_non_computer(folder_path * vehicle_params_path)
# ctrl_yaml = load_controller_params(folder_path * ctrl_yaml_path)
sim_params = load_sim_params(folder_path * sim_params_path, vehicle_params)

# create computer 
flight_controller = Computer.create_computer("stm32")
#@time Computer.position_controller(flight_controller, 10)

flight_controller.scheduler

include("integrator_callback.jl")

@time df = run_sim(sys, subsystems, sim_params, vehicle_params; save=false)
# @time df = run_sim_stepping1(sys, subsystems, sim_params, vehicle_params; save=false)

# plotting
FlyingRobots.Gui.set_sim_instance(plot_elements, df)

FlyingRobots.Gui.plot_position(plot_elements)
FlyingRobots.Gui.plot_orientation(plot_elements)
FlyingRobots.Gui.plot_control_input(plot_elements, motor_thrust_to_body_thrust)

flag = FlyingRobots.Gui.start_3d_animation(plot_elements)
end
