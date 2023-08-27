
function integrator_callback(int, sim_params=sim_params)

    dt::Float64 = sim_params.callback_dt
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
    int.u[7] = q1.q.s
    int.u[8] = q1.q.v1
    int.u[9] = q1.q.v2
    int.u[10] = q1.q.v3


    # update vehicle state ------------------------------------------------

    # set state estimate to ground truth value for now
    vehicle_pose = FlyingRobots.Pose3d7()

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

    flight_controller.ram_memory[:vehicle_pose] = vehicle_pose

    # # run the scheduler ------------------------------------------------
    motor_thrusts = Computer.scheduler(flight_controller, int.t)

    # set the control input
    c_index = 18

    int.u[c_index] = motor_thrusts[1]
    int.u[c_index+1] = motor_thrusts[2]
    int.u[c_index+2] = motor_thrusts[3]
    int.u[c_index+3] = motor_thrusts[4]
end


