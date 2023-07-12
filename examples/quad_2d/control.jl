export control_allocator, get_control_allocation_matrix

function get_control_allocation_matrix(quad_2d::Quad2D)
    L = quad_2d.params.L
    actuator_cmd_to_ctrl_cmd = @SMatrix[1 1; L -L]
    ctrl_cmd_to_actuator_cmd = inv(actuator_cmd_to_ctrl_cmd)
    return ctrl_cmd_to_actuator_cmd
end

function control_allocator(controller::Quad2DController, ctrl_cmd::Quad2DControlCmd)

    (left_motor_thrust, right_motor_thrust) = controller.allocation_matrix * [ctrl_cmd.body_thrust, ctrl_cmd.body_torque]
    actuator_cmd = Quad2DActuatorCmd(left_motor_thrust, right_motor_thrust)

end