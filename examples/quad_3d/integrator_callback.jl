
function quaternion_integrator!(integrator)
    # extract the state
    q0_vec = @view integrator.u[7:10]
    ω = @view integrator.u[11:13]

    dt = integrator.t - integrator.tprev

    # quaternion integration ------------------------------------------------
    q0 = QuatRotation(q0_vec, false)
    q1 = quaternion_integrator(q0, ω, dt)

    # set attitude
    integrator.u[7] = q1.q.s
    integrator.u[8] = q1.q.v1
    integrator.u[9] = q1.q.v2
    integrator.u[10] = q1.q.v3
end


function computer_cycle(int, sim_params=sim_params)

    dt::Float64 = sim_params.callback_dt
    g = 9.81

    # extract the state
    r = @view int.u[1:3]
    ṙ = @view int.u[4:6]
    # q0_vec = @view int.u[7:10]
    ω = @view int.u[11:13]

    # # quaternion integration ------------------------------------------------
    # q0 = QuatRotation(q0_vec, false)
    # q1 = quaternion_integrator(q0, ω, dt)

    # # set attitude
    # int.u[7] = q1.q.s
    # int.u[8] = q1.q.v1
    # int.u[9] = q1.q.v2
    # int.u[10] = q1.q.v3

    q1_vec = @view int.u[7:10]
    q1 = QuatRotation(q1_vec, false)


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


    # # get time elapsed since previous callback 
    # t_now = Dates.now()
    # elpased_time = t_now - sim_prop.prev_time
    # sim_prop.prev_time = sim_prop.prev_time

    # # subtract offset to match physical time
    # sleep_time =
    #     sleep(dt - 0.002)

    # if int.t > 5
    #     terminate!(int)
    #     print("Sim terminated ")
    # end
end


# let
#     # dt = Dates.unix2datetime(time())
#     # Dates.millisecond(dt)

#     t1 = Dates.now()

#     sleep(0.5)
#     t2 = Dates.now()
#     t2 - t1
# end