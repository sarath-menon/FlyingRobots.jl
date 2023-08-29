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

include("dynamics_utilities.jl")
include("mtk_models.jl")
include("types.jl")

include("dynamics.jl")
include("logging.jl")

include("sim.jl")
include("modelling.jl")
include("vehicle.jl")
include("computer.jl")

# read settings file 
folder_path = pwd() * "/examples/quad_3d"

vehicle_params_path = "/parameters/vehicle.yml"

sim_params_path = "/parameters/sim.yml"
ctrl_yaml_path = "/parameters/controller.yml"

vehicle_yaml = YAML.load_file(folder_path * vehicle_params_path; dicttype=Dict{Symbol,Any})
recursive_dict_to_namedtuple(vehicle_yaml[:computer])

vehicle_params = load_vehicle_params_non_computer(folder_path * vehicle_params_path)
# ctrl_yaml = load_controller_params(folder_path * ctrl_yaml_path)
sim_params = load_sim_params(folder_path * sim_params_path, vehicle_params)

# build system model (Thread 2 )
@time system_build_task = @tspawnat 2 build_system_model()

# show visualizer (Thread 1)
plot_elements = FlyingRobots.Gui.show_visualizer()

# create computer 
flight_controller = Computer.create_computer("stm32")
#@time Computer.position_controller(flight_controller, 10)

include("integrator_callback.jl")

#  get result from system build task 
sys, subsystems = fetch(system_build_task)

# running vizulizer, simulator and  onboard computer on single thread -----

# Full simulation
@time df = run_sim(sys, subsystems, sim_params, vehicle_params; save=false)

# Stepping simulation
@time df = run_sim_stepping(sys, subsystems, sim_params, vehicle_params; save=false)

# -----------------------------------------------------------------
# running vizulizer on 1st thread,(simulator+onboard computer) on 2nd thread

# Full simulation
@time sim_task = @tspawnat 2 run_sim(sys, subsystems, sim_params, vehicle_params; save=false)
df = fetch(sim_task)

# Stepping simulation
@time sim_task = @tspawnat 2 run_sim_stepping(sys, subsystems, sim_params, vehicle_params; save=false)
df = fetch(sim_task)

# -----------------------------------------------------------------
# # running vizulizer, simulator,onboard computer on 3 different threads
# c1 = Channel{Vector{Float64}}(10)
# condition = Threads.Condition()

# computer_task = @tspawnat 1 Computer.scheduler(flight_controller)
# @time sim_task = @tspawnat 2 run_sim(sys, subsystems, sim_params, vehicle_params; save=false)

# plotting
FlyingRobots.Gui.set_sim_instance(plot_elements, df)

FlyingRobots.Gui.plot_position(plot_elements)
FlyingRobots.Gui.plot_orientation(plot_elements)
FlyingRobots.Gui.plot_control_input(plot_elements, motor_thrust_to_body_thrust)

flag = FlyingRobots.Gui.start_3d_animation(plot_elements)
end
