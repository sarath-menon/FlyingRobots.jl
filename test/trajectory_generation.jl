## Trajectory generator test 

module FlyingRobotsTest
using Test

export run_tests

using FlyingRobots

# create system 
motor_left = create_frobj(BLDCMotor; thrust_min=0.0, thrust_max=12.5)
motor_right = create_frobj(BLDCMotor; thrust_min=0.0, thrust_max=12.5)

quad_obj = create_frobj(Quad2d; m=1.0, L=0.1, I_xx=0.003, motor_left, motor_right)

frmodel_params = create_frobj(FrModel; nx=6, nu=2, ny=3, Ts=0.01)

tspan = (0.0, 100.0)
dt = frmodel_params.Ts

circle_trajec = create_frobj(CircleTrajectory; r=1.0, ω=0.1 * π, y₀=2.0, z₀=2.0)

# time required to complete 1 revolution 
t_one_rev = (2 * π) / circle_trajec.ω

# get state at t =0
state_intial = generate_trajectory(circle_trajec, quad_obj, 0.0)

# get state after one revolution 
state_one_rev = generate_trajectory(circle_trajec, quad_obj, t_one_rev)

function run_tests()
    # checks whether both states are equal using floating point comparison 
    @test isapprox(state_intial, state_one_rev)
end

end
