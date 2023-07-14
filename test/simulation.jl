module SimulationTest

using FlyingRobots
using ControlSystems
using StaticArrays
using Test
using BenchmarkTools
using DifferentialEquations

include("./../examples/quad_2d/quad_2d.jl")

## Create objects 
motor_left = fr_create(BLDCMotor; thrust_min=0.0, thrust_max=12.5)
motor_right = fr_create(BLDCMotor; thrust_min=0.0, thrust_max=12.5)

quad_obj = fr_create(Quad2d; m=1.0, L=0.1, I_xx=0.003, motor_left, motor_right)
frmodel_params = fr_create(FrModel; nx=6, nu=2, ny=3, Ts=0.01)

safety_box = fr_create(SafetyBox; x_low=-10.0, x_high=10.0, y_low=-10.0, y_high=10.0, z_low=0.0, z_high=20.0)

## Linearization
x₀ = Pose2D(2, 1, 0, 0, 0, 0)

sys_c, sys_d, AB_symbolic = FlyingRobots.linearize_system(frmodel_params.Ts, x₀, quad_obj, [-g / 2, -g / 2])

##  Create an LQR controller
Q = Diagonal([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
R = Diagonal([1.0, 1.0])

dlqr_ctrl = FlyingRobots.create_lqr_controller(Q, R; params=frmodel_params, sys=sys_d)

# trajectory params
circle_trajec = fr_create(CircleTrajectory; r=1.0, ω=0.1 * π, y₀=2.0, z₀=2.0)

# vehicle parameters
quad_params = (; m=quad_obj.m, l=quad_obj.L, I_xx=0.003, safety_box=safety_box, K=dlqr_ctrl.K)

## setup simulation
tspan = (0.0, 60.0)

# logging parameters
n_rows::Int = length(tspan[1]:frmodel_params.Ts:tspan[2])
n_cols = 8
log_matrix = zeros(n_rows, n_cols + 1)
log_params = (; log_matrix=log_matrix)


# merged params 
params = (; quad=quad_params, trajectory=circle_trajec, frmodel=ntfromstruct(frmodel_params), logger=log_params)
quad2d_plot = quad2d_plot_initialize(frmodel_params, tspan)

#Initial Conditions
x₀ = Pose2D(3, 1, 0, 0, 0, 0)
initial_state = [x₀.y, x₀.z, x₀.θ, x₀.ẏ, x₀.ż, x₀.θ̇]
u₀ = [0, 0]

# initial_conditions : (initial state + intial contorl action)
initial_conditions = vcat(initial_state, u₀)

# setup ODE
prob = ODEProblem(dynamics_diffeq, initial_conditions, tspan, params, callback=control_cb)

# solve ODE
#sol = solve(prob, Tsit5(), abstol=1e-8, reltol=1e-8, save_everystep=false, save_on=false)
@time sol = solve(prob, Tsit5(), abstol=1e-8, reltol=1e-8, save_everystep=false)

# save solution to csv file
CSV.write("logs/log_no_alloc.csv", Tables.table(log_matrix), writeheader=false)

quad_2d_plot_normal(quad2d_plot, sol; y_ref=y_req, z_ref=z_req, theta_ref=θ_req)


@allocations solve(prob, Tsit5(), abstol=1e-8, reltol=1e-8, save_everystep=false, save_on=false)


@testset "Type Utities: Low level tests" begin
    
    # Test 1: Check if core dynamics funcs has zero allocations is zero 

    # Pre-run to compile 
    dynamics_diffeq(d_state, state, params, 0.1)
    
    allocations = @allocations dynamics_diffeq(d_state, state, params, 0.1)
    @test allocations ==0

    # Test 2: Check if simulation makes <100 allocations 

    # Pre-run to compile 
    solve(prob, Tsit5(), abstol=1e-8, reltol=1e-8, save_everystep=false, save_on=false)
    
    allocations = @allocations solve(prob, Tsit5(), abstol=1e-8, reltol=1e-8, save_everystep=false, save_on=false)
    @test allocations < 100

    # Test 3: Check whether the solve time is less than  50ms
    time_taken= @elapsed solve(prob, Tsit5(), abstol=1e-8, reltol=1e-8, save_everystep=false, save_on=false)
    @test time_taken <= 0.050
end