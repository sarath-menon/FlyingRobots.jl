using Revise
using LinearAlgebra
using GLMakie
using ControlSystems
using DifferentialEquations
using StaticArrays
using BenchmarkTools
using ForwardDiff
using NamedTupleTools
using ModelingToolkit

using CSV, DataFrames

GLMakie.activate!(inline=false)

include("types.jl")
include("utilities.jl")
include("linearize.jl")
include("plotting.jl")
include("sim.jl")
include("trajectory_generation.jl")

Revise.track("src/dynamics/utilities.jl")
Revise.track("src/dynamics/types.jl")
Revise.track("src/dynamics/linearize.jl")
Revise.track("src/dynamics/sim.jl")
Revise.track("src/dynamics/plotting.jl")
Revise.track("src/dynamics/trajectory_generation.jl")

## Create objects 

motor_left = create_frobj(BLDCMotor; thrust_min=0.0, thrust_max=12.5)
motor_right = create_frobj(BLDCMotor; thrust_min=0.0, thrust_max=12.5)

quad_obj = create_frobj(Quad2d; m=1.0, L=0.1, I_xx=0.003, motor_left, motor_right);

frmodel_params = create_frobj(FrModel; nx=6, nu=2, ny=3, Ts=0.01)

safety_box = create_frobj(SafetyBox; x_low=-10.0, x_high=10.0, y_low=-10.0, y_high=10.0, z_low=0.0, z_high=20.0);
## Linearization 

# equilibrium point
x₀ = Pose2D(2, 1, 0, 0, 0, 0)

const thrust_equilibirum::Float64 = 9.81;

const f_1_equilibirum::Float64 = thrust_equilibirum / 2
const f_2_equilibirum::Float64 = thrust_equilibirum / 2

sys_c, sys_d, AB_symbolic = linearize_system(frmodel_params.Ts, x₀, quad_obj, [f_1_equilibirum, f_2_equilibirum]);

## Non-linear Simulation with trajectory tracking using LQR

control_cb = PeriodicCallback(frmodel_params.Ts, initial_affect=true, save_positions=(false, true)) do integrator

    # # Extract the parameters
    # (; m, l, I_xx, safety_box, K) = integrator.p.quad
    # (; nx, nu, ny, Ts) = integrator.p.frmodel

    # # Extract the state 
    # X = @view integrator.u[1:nx]

    # X_req = generate_trajectory(circle_trajec, quad_params, integrator.t)

    # # compute control input
    # X_error = X - X_req
    # U = -K * X_error

    # # println("X_req: $(X_req)")
    # #println("X_error: $(X_error)")
    # #println("State: $(X)")

    # f_1::Float64 = f_1_equilibirum + U[1]
    # f_2::Float64 = f_2_equilibirum + U[2]

    # # constrain the control input
    # f_1 = clamp(f_1, quad_obj.motor_left.thrust_min, quad_obj.motor_left.thrust_max)
    # f_2 = clamp(f_2, quad_obj.motor_right.thrust_min, quad_obj.motor_right.thrust_max)

    # #Update the control-signal
    # U = [f_1, f_2]
    # integrator.u[frmodel_params.nx+1:end] .= SA_F64[f_1, f_2]

    # push!(df, (integrator.t, X, X_req, U))
end

#Initial Conditions
x₀ = Pose2D(3, 1, 0, 0, 0, 0)

##  Create LQR controller
Q = Diagonal([1.0, 1.0, 1.0, 1.0, 1.0, 1.0]);
R = Diagonal([1.0, 1.0]);
dlqr_ctrl = create_lqr_controller(Q, R; params=frmodel_params, sys=sys_d);

# # for logging
n_rows::Int = length(tspan[1]:frmodel_params.Ts:tspan[2])
n_cols = 3
log_matrix = zeros(n_rows, n_cols)
log_params = (; log_matrix=Ref(log_matrix))


# params
circle_trajec = create_frobj(CircleTrajectory; r=1.0, ω=0.1 * π, y₀=2.0, z₀=2.0)

# parameters
quad_params = (; m=quad_obj.m, l=quad_obj.L, I_xx=0.003, safety_box=safety_box, K=dlqr_ctrl.K);


params = (; quad=quad_params, frmodel=ntfromstruct(frmodel_params));

tspan = (0.0, 100.0);

initial_state = [x₀.y, x₀.z, x₀.θ, x₀.ẏ, x₀.ż, x₀.θ̇]; # state
u₀ = [0, 0]; # control

initial_conditions = vcat(initial_state, u₀);

#Pass to solvers
cb_set = CallbackSet(control_cb);

prob = ODEProblem(quad_2d, initial_conditions, tspan, params, callback=control_cb);
#prob = ODEProblem(quad_2d, initial_conditions, tspan, params);
sol = solve(prob, Tsit5(), abstol=1e-8, reltol=1e-8, save_everystep=false);

# compute entire reference trajectory at sol.t timesteps
#(y_req, z_req, θ_req, ẏ_req, ż_req, θ̇_req) = generate_trajectory(circle_trajec, quad_params, sol.t)

# save solution to csv file
# df = DataFrame(sol)
# CSV.write("logs/log1.csv", df)

#quad_2d_plot_normal(sol; y_ref=y_req, z_ref=z_req, theta_ref=θ_req)

## benchmarking 
@time solve(prob, Tsit5(), abstol=1e-8, reltol=1e-8, save_everystep=false);

# @time generate_trajectory(circle_trajec, quad_params, sol.t);
@time generate_trajectory(circle_trajec, params, 0.1);
