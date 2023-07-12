
## Dynamics test
module DynamicsTest

using FlyingRobots
using ControlSystems
using StaticArrays
using Test

include("./../examples/quad_2d/quad_2d.jl")
using .Quad2D_Demo

# testing
function run_tests()
    quad_2d_params = (; m=1.0, L=0.1, I_xx=0.003)
    initial_state = fr_create(Quad2DState; y=0.0, z=0.0, θ=0.0, ẏ=0.0, ż=0.0, θ̇=0.0)
    quad_2d = fr_create(Quad2D; nx=6, nu=2, state=initial_state, params=quad_2d_params)
    new_state = rand(quad_2d.nx)

    @testset "Type Dynamics: Low level tests" begin
        # test update state_func
        update_state!(quad_2d, new_state)
        @test check_if_struct_equals_vector(quad_2d.state, new_state)
    end

    initial_state = fr_create(Quad2DState; y=0.0, z=0.0, θ=0.0, ẏ=0.0, ż=0.0, θ̇=0.0)
    quad_2d = fr_create(Quad2D; nx=6, nu=2, state=initial_state, params=quad_2d_params)

    # ctrl_cmd = fr_create(Quad2DControlCmd; body_thrust=0.0, body_torque=0.0)
    # state_vec = dynamics!(quad_2d, ctrl_cmd)
    # ground_truth = [0.0, 0.0, 0.0, 0.0, g, 0.0,]

    @testset "Dynamics: dynamics function " begin

        # Test 1: 
        # Vehicle state: At rest on the ground
        # Control input: Thrust=0, Torque=0 
        # Expected outcome: vehicle stays in place, all zeros in state vector except for the acc due to gravity along z 
        ctrl_cmd = fr_create(Quad2DControlCmd; body_thrust=0.0, body_torque=0.0)
        state_vec = dynamics!(quad_2d, ctrl_cmd)
        ground_truth = [0.0, 0.0, 0.0, 0.0, g, 0.0,]

        @test isapprox(state_vec, ground_truth)

        # Test 2: 
        # Vehicle state: Hovering at altitude of 1 m
        # Control input: thrust=-g, torque=0
        # Expected state: vehicle stays in place, all zeros in state vector

        ctrl_cmd = fr_create(Quad2DControlCmd; body_thrust=-g, body_torque=0.0)
        state_vec = dynamics!(quad_2d, ctrl_cmd)
        ground_truth = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        @test isapprox(state_vec, ground_truth)

    end



end

run_tests()

end