
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
    L = quad_2d_params.L
    initial_state = Quad2DState(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    quad_2d = Quad2D(6, 2, initial_state, quad_2d_params)

    allocation_matrix = get_control_allocation_matrix(quad_2d)
    quad_2d_controller = Quad2DController(allocation_matrix)

    @testset "Type Control Allocator" begin
        # Test 1: 
        # Control cmd: Thrust=0, Torque=0 
        # Expected actuator cmd: left_motor_thrust=0, right_motor_thrust=0
        ctrl_cmd = Quad2DControlCmd(0.0, 0.0)
        actuator_cmd = control_allocator(quad_2d_controller, ctrl_cmd)

        ground_truth = [0.0, 0.0]
        actator_vec = [actuator_cmd.left_motor_thrust, actuator_cmd.right_motor_thrust]

        @test isapprox(actator_vec, ground_truth)

        # Test 2: 
        # Control cmd: thrust=1.0, torque=0 
        # Expected actuator cmd: left_motor_thrust=(thrust/2), right_motor_thrust=(thrust/2)
        ctrl_cmd = Quad2DControlCmd(1.0, 0.0)
        actuator_cmd = control_allocator(quad_2d_controller, ctrl_cmd)

        ground_truth = [ctrl_cmd.body_thrust / 2, ctrl_cmd.body_thrust / 2]
        actator_vec = [actuator_cmd.left_motor_thrust, actuator_cmd.right_motor_thrust]

        @test isapprox(actator_vec, ground_truth)

        # Test 3: 
        # Control cmd: body_thrust=0.0, body_torque=1.0 
        # Expected outcome: 
        #a) left_motor_thrust + right_motor_thrust == body_thrust
        #b) (left_motor_thrust - right_motor_thrust) * L == body_torque

        ctrl_cmd = Quad2DControlCmd(0.0, 1.0)
        actuator_cmd = control_allocator(quad_2d_controller, ctrl_cmd)

        # ground_truth = [ctrl_cmd.body_thrust / (2 * L), -ctrl_cmd.body_thrust / (2 * L)]
        # actator_vec = [actuator_cmd.left_motor_thrust, actuator_cmd.right_motor_thrust]

        @test isapprox(actuator_cmd.left_motor_thrust + actuator_cmd.right_motor_thrust, 0.0)
        @test isapprox((actuator_cmd.left_motor_thrust - actuator_cmd.right_motor_thrust) * L, 1.0)
    end
end

run_tests()

end