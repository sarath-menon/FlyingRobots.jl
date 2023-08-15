export control_allocator, get_control_allocation_matrix
export actuator_cmd_to_ctrl_cmd
export control_cb

using DifferentialEquations

control_cb = PeriodicCallback(0.01, initial_affect=true, final_affect=true, save_positions=(false, false)) do integrator

    # Extract the parameters
    (; m, l, I_xx, safety_box, K) = integrator.p.quad
    nx, nu, ny, Ts = integrator.p.frmodel
    # (; log_matrix) = integrator.p.logger
    logger = integrator.p.logger

    # Extract the state 
    X = @view integrator.u[1:nx]

    X_req = generate_trajectory(circle_trajec, quad_obj, integrator.t)

    # compute control input
    X_error = X - X_req
    U = -K * X_error

    # # println("X_req: $(X_req)")
    # #println("X_error: $(X_error)")
    #println("State: $(X)")

    f_1_equilibirum = f_2_equilibirum = -g / 2

    f_1::Float64 = f_1_equilibirum + U[1]
    f_2::Float64 = f_2_equilibirum + U[2]

    # constrain the control input
    f_1 = clamp(f_1, 0.0, 12.5)
    f_2 = clamp(f_2, 0.0, 12.5)

    #Update the control-signal
    integrator.u[nx+1:end] = @SVector [f_1, f_2]

    # logging
    # write!(logger, integrator.u, integrator.t, Ts; start_index=1)
    write!(logger, integrator.u, integrator.t)

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