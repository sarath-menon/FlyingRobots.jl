
## Dynamics test
module ControllerTest

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

    allocation_matrix = get_control_allocation_matrix(quad_2d)
    quad_2d_controller = Quad2DController(allocation_matrix)

    # Test 1: 
    # Control cmd: Thrust=0, Torque=0 
    # Expected actuator cmd: left_motor_thrust=0, right_motor_thrust=0
    ctrl_cmd = Quad2DControlCmd(0.0, 0.0)
    actuator_cmd = control_allocator(quad_2d_controller, ctrl_cmd)

    ground_truth = [0.0, 0.0]
    actator_vec = [actuator_cmd.left_motor_thrust, actuator_cmd.right_motor_thrust]

    @testset "Type Control Allocator: Low level tests" begin
        @test isapprox(actator_vec, ground_truth)
    end
end

run_tests()

end