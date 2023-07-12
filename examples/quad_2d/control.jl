export control_allocator, get_control_allocation_matrix
export actuator_cmd_to_ctrl_cmd

actuator_cmd_to_ctrl_cmd_matrix(L::Float64) = @SMatrix[1 1; L -L]

function actuator_cmd_to_ctrl_cmd(quad_2d::Quad2D, actuator_cmd::Quad2DActuatorCmd)

    # extract the parameters
    m, L, I_xx = quad_2d.params

    # f_left = actuator_cmd.left_motor_thrust
    # f_right = actuator_cmd.right_motor_thrust

    actuator_cmd_vec = @SVector[actuator_cmd.left_motor_thrust, actuator_cmd.right_motor_thrust]

    # # compute body  thrust
    # body_thrust = f_left + f_right
    # mass_normalized_body_thrust = (body_thrust / m) # mass normalized body  thrust 

    # # compute body thrust 
    # body_torque = (f_left - f_right) * L

    control_cmd_vec = actuator_cmd_to_ctrl_cmd_matrix(L) * actuator_cmd_vec
    control_cmd = fr_create(Quad2DControlCmd; body_thrust=control_cmd_vec[1], body_torque=control_cmd_vec[2])

    return control_cmd
end


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