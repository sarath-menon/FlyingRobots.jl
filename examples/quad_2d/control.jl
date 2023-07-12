export control_allocator, get_control_allocation_matrix
export actuator_cmd_to_ctrl_cmd



function get_control_allocation_matrix(quad_2d::Quad2D)
    L = quad_2d.params.L

    ctrl_cmd_to_actuator_cmd_matrix = inv(actuator_cmd_to_ctrl_cmd_matrix(L))
    return ctrl_cmd_to_actuator_cmd_matrix
end

function ctrl_cmd_to_actuator_cmd(controller::Quad2DController, ctrl_cmd::Quad2DControlCmd)

    ctrl_cmd_vec = @SVector[ctrl_cmd.body_thrust, ctrl_cmd.body_torque]

    (left_motor_thrust, right_motor_thrust) = controller.allocation_matrix * ctrl_cmd_vec
    actuator_cmd = Quad2DActuatorCmd(left_motor_thrust, right_motor_thrust)

    return actuator_cmd
end

control_allocator = ctrl_cmd_to_actuator_cmd