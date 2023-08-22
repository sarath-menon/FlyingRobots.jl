


callback_params = (; allocation_matrix=body_thrust_to_motor_thrust(0.7, 0.0035),
    reference_generator=reference_generator)

function digital_controller(int; params=callback_params)

    # extract the state
    r = @view int.u[1:3]
    ṙ = @view int.u[4:6]
    q0_vec = @view int.u[7:10]
    ω = @view int.u[11:13]

    dt = 0.01
    m = 1.0
    g = 9.81

    # quaternion integration ------------------------------------------------
    q0 = QuatRotation(q0_vec, false)
    q1 = quaternion_integrator(q0, ω, dt)

    # set attitude
    int.u[7:10] = [q1.q.s, q1.q.v1, q1.q.v2, q1.q.v3]

    # controller ------------------------------------------------

    allocation_matrix = params.allocation_matrix

    # get reference 
    R = reference_generator(int.t)

    e_x = R[1] - r[1]
    e_y = R[2] - r[2]
    e_z = R[3] - r[3]

    # control laws 

    # x position controller
    q_ref = pid_controller(x_pos_pid; e=e_x, umin=-0.5, umax=0.5) / g

    # y position controller
    p_ref = -pid_controller(y_pos_pid; e=e_y, umin=-0.5, umax=0.5) / g

    # z position controller
    f_net = m * (g + pid_controller(z_pos_pid; e=e_z, umin=-100, umax=100))

    # attitude controller

    # convert quaternion attitude representation to euler angles 
    r, q, p = Rotations.params(RotZYX(q1))

    e_p = p_ref - p
    e_q = q_ref - q

    τ_x = pid_controller(roll_pid; e=e_p, umin=-25, umax=25)
    τ_y = pid_controller(pitch_pid; e=e_q, umin=-25, umax=25)
    τ_z = 0

    # f_net = 9.81
    # τ_x, τ_y= 0,0

    motor_thrusts = allocation_matrix * [f_net; τ_x; τ_y; τ_z]

    # set the control input
    c_index = 14
    int.u[c_index:c_index+3] .= motor_thrusts

    # @show R_IB.q
    # @show f_net_I

    # @show e_x,e_y,e_z 
    # # @show p, q
    # # @show p_ref, q_ref
    # @show e_p,e_q
    # @show  τ_x, τ_y
    # @show motor_thrusts 
    # println("")    
end

function reference_generator(t)

    r = 0.5    # circle radius 
    ω = 0.2    # angular velocity

    z_0 = 1

    # circular trajectory 
    x_ref = r * sin(ω * t)
    y_ref = r * sin(ω * t)
    z_ref = 1.0

    # x_ref = 1
    # y_ref = 2.
    # z_ref = 1.0

    return [x_ref, y_ref, z_ref]
end

