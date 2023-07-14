module quad_2d_test

using Revise
using FlyingRobots
using LinearAlgebra
using NamedTupleTools
using DifferentialEquations
using Tables, CSV
GLMakie.activate!(inline=false)

## Create objects 
motor_left = fr_create(BLDCMotor; thrust_min=0.0, thrust_max=12.5)
motor_right = fr_create(BLDCMotor; thrust_min=0.0, thrust_max=12.5)

quad_obj = fr_create(Quad2d; m=1.0, L=0.1, I_xx=0.003, motor_left, motor_right)
frmodel_params = fr_create(FrModel; nx=6, nu=2, ny=3, Ts=0.01)

safety_box = fr_create(SafetyBox; x_low=-10.0, x_high=10.0, y_low=-10.0, y_high=10.0, z_low=0.0, z_high=20.0)


## Linearization
x₀ = Pose2D(2, 1, 0, 0, 0, 0)


sys_c, sys_d, AB_symbolic = FlyingRobots.linearize_system(frmodel_params.Ts, x₀, quad_obj, [-g / 2, -g / 2])

## initialize simulation
tspan = (0.0, 60.0)

#Initial Conditions
x₀ = Pose2D(3, 1, 0, 0, 0, 0)

##  Create an LQR controller
Q = Diagonal([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
R = Diagonal([1.0, 1.0])

dlqr_ctrl = FlyingRobots.create_lqr_controller(Q, R; params=frmodel_params, sys=sys_d)

# trajectory params
circle_trajec = fr_create(CircleTrajectory; r=1.0, ω=0.1 * π, y₀=2.0, z₀=2.0)

# vehicle parameters
quad_params = (; m=quad_obj.m, l=quad_obj.L, I_xx=0.003, safety_box=safety_box, K=dlqr_ctrl.K)

# logging parameters
n_rows::Int = length(tspan[1]:frmodel_params.Ts:tspan[2])
n_cols = 8
log_matrix = zeros(n_rows, n_cols + 1)
log_params = (; log_matrix=log_matrix)

# merged params 
params = (; quad=quad_params, trajectory=circle_trajec, frmodel=ntfromstruct(frmodel_params), logger=log_params)


initial_state = [x₀.y, x₀.z, x₀.θ, x₀.ẏ, x₀.ż, x₀.θ̇]
u₀ = [0, 0]

# initial_conditions : (initial state + intial contorl action)
initial_conditions = vcat(initial_state, u₀)

# Period callback that applies discrete controller 
control_cb = setup_control_cb(frmodel_params, quad_obj)

# setup ODE
prob = ODEProblem(quad_2d_dynamics_diffeq, initial_conditions, tspan, params, callback=control_cb)
#prob = ODEProblem(quad_2d_dynamics_diffeq, initial_conditions, tspan, params)

# solve ODE
#sol = solve(prob, Tsit5(), abstol=1e-8, reltol=1e-8, save_everystep=false, save_on=false)
sol = solve(prob, Tsit5(), abstol=1e-8, reltol=1e-8, save_everystep=false)
#FlyingRobots.simulate(; X_0=initial_conditions, tspan=tspan, params=params, control_cb=control_cb)

#compute reference trajectory for entire duration of the simulation
(y_req, z_req, θ_req, ẏ_req, ż_req, θ̇_req) = generate_trajectory(circle_trajec, quad_obj, sol.t)

# save solution to csv file
CSV.write("logs/log_no_alloc.csv", Tables.table(log_matrix), writeheader=false)

quad2d_plot = quad2d_plot_initialize(frmodel_params, tspan)
quad_2d_plot_normal(quad2d_plot, sol; y_ref=y_req, z_ref=z_req, theta_ref=θ_req)


# benchmarking 
@time sol = solve(prob, Tsit5(), abstol=1e-8, reltol=1e-8, save_everystep=false, save_on=false)
#@time sol = solve(prob, Tsit5(), abstol=1e-8, reltol=1e-8, save_everystep=false);

end