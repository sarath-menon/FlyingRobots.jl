

function reference_generator(t)

    r = 0.5    # circle radius 
    ω = 0.2    # angular velocity

    z_0 = 1

    # circular trajectory 
    x_ref = r * sin(ω * t)
    y_ref = r * sin(ω * t)
    z_ref = 1.0

    # x_ref = 0.0
    # y_ref = 0.0
    # z_ref = 1.0

    return [x_ref, y_ref, z_ref]
end


# vehicle_pose = Pose3d()
ctrl_cmd = CascadedPidCtrlCmd()
vehicle_pose = Pose3d()

callback_params = (; reference_generator=reference_generator, ctrl_cmd=ctrl_cmd, vehicle_pose=vehicle_pose, vehicle_params=vehicle_params)

function digital_controller(int; params=callback_params)

    dt = 0.01
    m = 1.0
    g = 9.81

    # extract the state
    r = @view int.u[1:3]
    ṙ = @view int.u[4:6]
    q0_vec = @view int.u[7:10]
    ω = @view int.u[11:13]

    # quaternion integration ------------------------------------------------
    q0 = QuatRotation(q0_vec, false)
    q1 = quaternion_integrator(q0, ω, dt)

    # set attitude
    int.u[7:10] = [q1.q.s, q1.q.v1, q1.q.v2, q1.q.v3]

    # update vehicle state
    vehicle_params = params.vehicle_params
    vehicle_pose = params.vehicle_pose
    ctrl_cmd = params.ctrl_cmd

    vehicle_pose.pos.x = r[1]
    vehicle_pose.pos.y = r[2]
    vehicle_pose.pos.z = r[3]

    vehicle_pose.orientation = q1

    vehicle_pose.vel.x = ṙ[1]
    vehicle_pose.vel.y = ṙ[2]
    vehicle_pose.vel.z = ṙ[3]

    vehicle_pose.angular_vel.x = ω[1]
    vehicle_pose.angular_vel.y = ω[2]
    vehicle_pose.angular_vel.z = ω[3]

    # get the clock count from simulation time
    clock = round(Int, int.t / dt) + 1

    # get reference 
    R = reference_generator(int.t)

    ctrl_cmd.x = R[1]
    ctrl_cmd.y = R[2]
    ctrl_cmd.z = R[3]

    # run the scheduler
    scheduler(clock, vehicle_pose, ctrl_cmd, vehicle_params)

    # controller ------------------------------------------------
    # e_x = R[1] - r[1]
    # e_y = R[2] - r[2]
    # e_z = R[3] - r[3]

    # control laws 

    # # x position controller
    # ctrl_cmd.q = pid_controller(x_pos_pid; e=e_x, umin=-0.5, umax=0.5) / g

    # # y position controller
    # ctrl_cmd.p = -pid_controller(y_pos_pid; e=e_y, umin=-0.5, umax=0.5) / g

    # # z position controller
    # ctrl_cmd.f_net = m * (g + pid_controller(z_pos_pid; e=e_z, umin=-4.5, umax=40.0))

    # attitude controller

    # # convert quaternion attitude representation to euler angles 
    # r, q, p = Rotations.params(RotZYX(q1))

    # e_p = ctrl_cmd.p - p
    # e_q = ctrl_cmd.q - q

    # τ_x = pid_controller(roll_pid; e=e_p, umin=-25, umax=25)
    # τ_y = pid_controller(pitch_pid; e=e_q, umin=-25, umax=25)
    # τ_z = 0

    # f_net = 9.81
    # τ_x, τ_y= 0,0

    # motor_thrusts = ctrl_yaml[:allocation_matrix] * [ctrl_cmd.f_net; ctrl_cmd.τ_x; ctrl_cmd.τ_y; ctrl_cmd.τ_z]

    # set the control input
    c_index = 18
    # int.u[c_index:c_index+3] .= motor_thrusts
    int.u[c_index:c_index+3] .= ctrl_cmd.motor_thrusts

    # @show R_IB.q
    # @show ctrl_cmd.f_net

    # @show e_x,e_y,e_z 
    # # @show p, q
    # # @show p_ref, q_ref
    # @show e_p,e_q
    # @show  τ_x, τ_y
    # @show motor_thrusts 
    # println("")    
end

function reset_pid_controllers()
    pid_reset(x_pos_pid)
    pid_reset(y_pos_pid)
    pid_reset(z_pos_pid)

    pid_reset(roll_pid)
    pid_reset(pitch_pid)
end

