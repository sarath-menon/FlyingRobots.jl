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
include("integrator_callback.jl")
include("vehicle.jl")

# read settings file 
folder_path = pwd() * "/examples/quad_3d"

# vehicle_yaml = YAML.load_file(folder_path * "/parameters/vehicle.yml"; dicttype=Dict{Symbol,Any})
# ctrl_yaml = YAML.load_file(folder_path * "/parameters/controller.yml"; dicttype=Dict{Symbol,Any})

ctrl_yaml = load_controller_params("/parameters/controller.yml")
vehicle_params = load_vehicle_params("/parameters/vehicle.yml")
sim_params = load_sim_params("/parameters/sim.yml")
# sim_yaml = YAML.load_file(folder_path * "/parameters/sim.yml"; dicttype=Dict{Symbol,Any})

# vehicle_params = recursive_dict_to_namedtuple(vehicle_yaml)


task_rates = vehicle_params.computer.task_rates
tasks_per_ticks = get_ticks_per_task(task_rates)

# # set integrator callback rate
# sim_yaml[:callback_dt] = 1 / vehicle_yaml[:computer][:clock_speed]

# sim_params = recursive_dict_to_namedtuple(sim_yaml)



## initialize subsystems
# @named plant = Quadcopter(; name=:quad1, l=0.7, k_τ=0.0035, m=1.0, I_xx=0.003, I_yy=0.003, I_zz=0.02)
@named plant = Quadcopter(; name=:quad1)
@named controller = Controller_Zero_Order_Hold()

# motor thrusts
eqn1 = controller.U .~ plant.f_cmd

# connect the subsystems
eqns = vcat(eqn1)
@named model = ODESystem(eqns,
    systems=[plant, controller])

sys = structural_simplify(model)

# get system properties
# states(sys)
# ModelingToolkit.get_ps(sys)

# set controller params 
allocation_matrix = body_thrust_to_motor_thrust(vehicle_params.arm_length, vehicle_params.actuators.constants.k_τ)
ctrl_yaml[:allocation_matrix] = allocation_matrix
ctrl_yaml[:position_controller][:rate] = 1 / vehicle_params.computer.task_rates.pos_ctrl_loop
ctrl_yaml[:attitude_controller][:rate] = 1 / vehicle_params.computer.task_rates.attitude_ctrl_loop

## controllers
x_pos_pid = PID(ctrl_yaml[:position_controller][:pid_x]; Ts=ctrl_yaml[:position_controller][:rate])
y_pos_pid = PID(ctrl_yaml[:position_controller][:pid_y]; Ts=ctrl_yaml[:position_controller][:rate])
z_pos_pid = PID(ctrl_yaml[:position_controller][:pid_z]; Ts=ctrl_yaml[:position_controller][:rate])

roll_pid = PID(ctrl_yaml[:attitude_controller][:pid_roll]; Ts=ctrl_yaml[:attitude_controller][:rate])
pitch_pid = PID(ctrl_yaml[:attitude_controller][:pid_pitch]; Ts=ctrl_yaml[:attitude_controller][:rate])

control_callback = PeriodicCallback(integrator_callback, sim_params.callback_dt, initial_affect=true)

## Simulation

tspan = (0.0, 60.0)

# initial conditions
r₀ = [0.0, 0.0, 1.0] # position
ṙ₀ = [0.0, 0.0, 0.0] # velocity
q₀ = QuatRotation(RotZYX(0.0, 0.0, 0.0)) # (Orientation - yaw, pitch, roll)
ω₀ = [0.0, 0.0, 0.0] # angular velocity

X₀ = [collect(plant.rb.r .=> r₀)
    collect(plant.rb.ṙ .=> ṙ₀)
    collect(plant.rb.q .=> [q₀.q.s, q₀.q.v1, q₀.q.v2, q₀.q.v3])
    collect(plant.rb.ω .=> ω₀)]

parameters = [
    plant.rb.m => vehicle_params.mass,
    plant.l => vehicle_params.arm_length,
    plant.k_τ => vehicle_params.actuators.constants.k_τ,
    plant.rb.I_xx => vehicle_params.I_xx,
    plant.rb.I_yy => vehicle_params.I_yy,
    plant.rb.I_zz => vehicle_params.I_zz,
    plant.motor_1.first_order_system.T => vehicle_params.actuators.constants.τ,
    plant.motor_2.first_order_system.T => vehicle_params.actuators.constants.τ,
    plant.motor_3.first_order_system.T => vehicle_params.actuators.constants.τ,
    plant.motor_4.first_order_system.T => vehicle_params.actuators.constants.τ]

states(sys)


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