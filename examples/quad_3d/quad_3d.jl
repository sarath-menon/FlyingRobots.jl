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
include("mtk_models.jl")
include("types.jl")

include("dynamics.jl")
# include("controller.jl")
include("plotting.jl")
include("logging.jl")

include("scheduler.jl")
# include("vehicle.jl")
include("sim.jl")
include("modelling.jl")
include("vehicle.jl")
include("computer.jl")



# read settings file 
folder_path = pwd() * "/examples/quad_3d"

vehicle_params_path = "/parameters/vehicle.yml"

sim_params_path = "/parameters/sim.yml"

# build system model
sys, subsystems = build_system_model()

vehicle_params = load_vehicle_params(folder_path * vehicle_params_path)
# ctrl_yaml = load_controller_params(folder_path * ctrl_yaml_path)
sim_params = load_sim_params(folder_path * sim_params_path, vehicle_params)


include("integrator_callback.jl")

@time sol, df = run_sim(sys, subsystems, sim_params, vehicle_params; save=true)

# plotting

plot_elements = FlyingRobots.Gui.show_visualizer()

FlyingRobots.Gui.set_sim_instance(plot_elements, df)

flag = FlyingRobots.Gui.start_3d_animation(plot_elements)

FlyingRobots.Gui.plot_position(plot_elements)
FlyingRobots.Gui.plot_orientation(plot_elements)
FlyingRobots.Gui.plot_control_input(plot_elements, motor_thrust_to_body_thrust)
end