module quad_3d

using ModelingToolkit
using DiffEqCallbacks
using LinearAlgebra
using OrdinaryDiffEq
using Rotations
using NamedTupleTools
using StaticArrays

using GLMakie
using BenchmarkTools
using YAML

GLMakie.activate!(inline=false)

using FlyingRobots
using MtkLibrary

include("dynamics_utilities.jl")
include("controller_utilities.jl")
include("types.jl")

include("dynamics.jl")
include("controller.jl")
include("plotting.jl")
include("gui.jl")
include("gui_utilities.jl")
include("scheduler.jl")
include("vehicle.jl")
include("sim.jl")
include("modelling.jl")

# read settings file 
folder_path = pwd() * "/examples/quad_3d"

ctrl_yaml = load_controller_params("/parameters/controller.yml")
vehicle_params = load_vehicle_params("/parameters/vehicle.yml")
sim_params = load_sim_params("/parameters/sim.yml", vehicle_params)

include("integrator_callback.jl")

# build system model
sys = build_system_model()

# set controller params 
allocation_matrix = body_thrust_to_motor_thrust(vehicle_params.arm_length, vehicle_params.actuators.constants.k_τ)
ctrl_yaml[:allocation_matrix] = allocation_matrix

control_callback = PeriodicCallback(integrator_callback, sim_params.callback_dt, initial_affect=true)

## Simulation
tspan = (0.0, sim_params.duration)

X₀ = get_initial_conditions(vehicle_params)
parameters = get_parameters(vehicle_params)

reset_pid_controllers()
prob = ODEProblem(sys, X₀, tspan, parameters, callback=control_callback)
@time sol = solve(prob, Tsit5(), abstol=1e-8, reltol=1e-8, save_everystep=false)

# plotting
# plot_position_attitude(sol)
state_indices = (position=(1, 3), orientation=(7, 10), motor_thrusts=(17, 21))

Gui.plot_reset(Gui.plot_data)

plot_position(Gui.state_plots, Gui.plot_data, sol, state_indices)
# @time plot_orientation(Gui.state_plots, Gui.plot_data, sol, state_indices)

plot_control_input(Gui.control_plots, Gui.plot_data, sol, state_indices, vehicle_params)

end