module quad_3d_stepping

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
# include("controller.jl")
include("logging.jl")

# include("scheduler.jl")
# include("vehicle.jl")
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

include("integrator_stepping_callback.jl")

# simulation
# quaternion integrator callback 
condition(u, t, integrator) = true

function quaternion_integrator!(integrator)
    # extract the state
    q0_vec = @view integrator.u[7:10]
    ω = @view integrator.u[11:13]

    dt = integrator.t - integrator.tprev

    # quaternion integration ------------------------------------------------
    q0 = QuatRotation(q0_vec, false)
    q1 = quaternion_integrator(q0, ω, dt)

    # set attitude
    integrator.u[7] = q1.q.s
    integrator.u[8] = q1.q.v1
    integrator.u[9] = q1.q.v2
    integrator.u[10] = q1.q.v3
end

function run_sim_stepping(sys, subsystems, sim_params, vehicle_params; tspan)

    control_cb = PeriodicCallback(computer_cycle, sim_params.callback_dt, initial_affect=true, save_positions=(true, true))
    integrator_cb = DiscreteCallback(condition, quaternion_integrator!, save_positions=(false, false))

    cb_set = CallbackSet(integrator_cb, control_cb)

    X₀ = get_initial_conditions(subsystems.plant, vehicle_params)
    parameters = get_parameters(subsystems.plant, vehicle_params)

    prob = ODEProblem(sys, X₀, tspan, parameters, callback=cb_set, save_everystep=false)

    integrator = init(prob, Tsit5(), abstol=1e-8, reltol=1e-8)


    while integrator.t < tspan[2]
        step!(integrator)
    end

    df = DataFrame(integrator.sol)
    # dataframe header formatting
    rename!(df, replace.(names(df), r"getindex" => ""))
    rename!(df, replace.(names(df), r"₊" => "."))

    return df
end

@time df = run_sim_stepping(sys, subsystems, sim_params, vehicle_params; tspan=(0.0, 60))

# plotting

FlyingRobots.Gui.set_sim_instance(plot_elements, df)

FlyingRobots.Gui.plot_position(plot_elements)
FlyingRobots.Gui.plot_orientation(plot_elements)
FlyingRobots.Gui.plot_control_input(plot_elements, motor_thrust_to_body_thrust)

flag = FlyingRobots.Gui.start_3d_animation(plot_elements)
end
