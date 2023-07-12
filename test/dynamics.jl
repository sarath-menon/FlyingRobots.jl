
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
    initial_state = Quad2DState(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    quad_2d = Quad2D(6, 2, initial_state, quad_2d_params)
    new_state = rand(quad_2d.nx)

    @testset "Type Dynamics: Low level tests" begin
        # test update state_func
        update_state!(quad_2d, new_state)
        @test check_if_struct_equals_vector(quad_2d.state, new_state)
    end

    quad_2d_params = (; m=1.0, L=0.1, I_xx=0.003)
    initial_state = Quad2DState(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    quad_2d = Quad2D(6, 2, initial_state, quad_2d_params)

    ctrl_cmd = Quad2DControlCmd(0.0, 0.0)
    state_vec = dynamics!(quad_2d, ctrl_cmd)
    ground_truth = [0.0, 0.0, 0.0, 0.0, -9.81, 0.0,]

    @testset "Type Utities: High level functions tests" begin

        # Test 1: When the vehicle is initially at rest and control control cmd is zero, only force acting on the vehicle should be acc due to gravity 
        @test isapprox(state_vec, ground_truth)
    end

end

run_tests()

end