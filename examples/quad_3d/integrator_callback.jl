

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

callback_params = (; ctrl_cmd=ctrl_cmd, vehicle_pose=vehicle_pose)

function integrator_callback(int; params=callback_params)

    dt = sim_params.callback_dt
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

    # update vehicle state ------------------------------------------------
    # vehicle_params = params.vehicle_params
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

    ctrl_cmd.pos.x = R[1]
    ctrl_cmd.pos.y = R[2]
    ctrl_cmd.pos.z = R[3]

    # run the scheduler ------------------------------------------------
    scheduler(clock, vehicle_pose, ctrl_cmd, vehicle_params)

    # set the control input
    c_index = 18
    # int.u[c_index:c_index+3] .= motor_thrusts
    int.u[c_index:c_index+3] .= ctrl_cmd.motor_thrusts
end


