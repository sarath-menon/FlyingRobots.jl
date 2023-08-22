module quad_3d

using ModelingToolkit
using DiffEqCallbacks
using LinearAlgebra
using OrdinaryDiffEq
using Rotations

using GLMakie
using BenchmarkTools
using GeometryBasics
using FileIO
using YAML

GLMakie.activate!(inline=false)

using FlyingRobots

include("dynamics_utilities.jl")
include("controller_utilities.jl")
include("gui_helper.jl")

include("dynamics.jl")
include("controller.jl")
include("plotting.jl")
include("gui.jl")


# read settings file 
folder_path = pwd() * "/examples/quad_3d"
plot_yaml = YAML.load_file(folder_path * "/parameters/plot.yml"; dicttype=Dict{Symbol,Any})
sim_yaml = YAML.load_file(folder_path * "/parameters/sim.yml"; dicttype=Dict{Symbol,Any})

plot_params = recursive_dict_to_namedtuple(plot_yaml)
sim_params = recursive_dict_to_namedtuple(sim_yaml)

# initialize subsystems
@named plant = Quadcopter(; name=:quad1, l=0.7, k_τ=0.0035, m=1.0, I_xx=0.003, I_yy=0.003, I_zz=0.02)
@named controller = Controller_Zero_Order_Hold()

# motor thrusts
eqn1 = controller.U .~ plant.f

# connect the subsystems
eqns = vcat(eqn1)
@named model = ODESystem(eqns,
    systems=[plant, controller])

sys = structural_simplify(model)


# controllers
x_pos_pid = PID(; kp=3.5, ki=0.00, kd=7, k_aw=0.0, Ts=0.01)
y_pos_pid = PID(; kp=3.5, ki=0.00, kd=7, k_aw=0.0, Ts=0.01)
z_pos_pid = PID(; kp=1.7, ki=0.1, kd=2.0, k_aw=0.0, Ts=0.01)

roll_pid = PID(; kp=0.05, ki=0.00, kd=0.07, k_aw=0.0, Ts=0.01)
pitch_pid = PID(; kp=0.05, ki=0.00, kd=0.07, k_aw=0.0, Ts=0.01)

control_callback = PeriodicCallback(digital_controller, 0.01, initial_affect=true)


## Simulation

tspan = (0.0, 60.0)

# initial conditions
r₀ = [0.0, 0.0, 0.0] # position
ṙ₀ = [0.0, 0.0, 0.0] # velocity
q₀ = QuatRotation(RotZYX(0.0, 0.0, 0.0)) # (Orientation - yaw, pitch, roll)
ω₀ = [0.0, 0.0, 0.0] # angular velocity

X₀ = [collect(plant.rb.r .=> r₀)
    collect(plant.rb.ṙ .=> ṙ₀)
    collect(plant.rb.q .=> [q₀.q.s, q₀.q.v1, q₀.q.v2, q₀.q.v3])
    collect(plant.rb.ω .=> ω₀)]

prob = ODEProblem(sys, X₀, tspan, callback=control_callback)
@time sol = solve(prob, Tsit5(), abstol=1e-8, reltol=1e-8, save_everystep=false)


# plotting
plot_position_attitude(sol)

end