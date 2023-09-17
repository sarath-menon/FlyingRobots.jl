
using FlyingRobots.Dynamics: quaternion_integrator

function integrator_callback(integrator)
    # extract the state
    q_vec = @view integrator.u[7:10]
    ω = @view integrator.u[11:13]

    dt = integrator.t - integrator.tprev

    # quaternion integration ------------------------------------------------
    q = QuatRotation(q_vec, false)
    q1 = quaternion_integrator(q, ω, dt)

    # set attitude
    integrator.u[7] = q1.q.s
    integrator.u[8] = q1.q.v1
    integrator.u[9] = q1.q.v2
    integrator.u[10] = q1.q.v3
end


function computer_cycle(int)

    g = 9.81

    # extract the state
    r = @view int.u[1:3]
    ṙ = @view int.u[4:6]
    q_vec = @view int.u[7:10]
    ω = @view int.u[11:13]

    q = QuatRotation(q_vec, false)

    # update vehicle state ------------------------------------------------

    # set state estimate to ground truth value for now
    vehicle_pose = FlyingRobots.Pose3d7()

    vehicle_pose.pos.x = r[1]
    vehicle_pose.pos.y = r[2]
    vehicle_pose.pos.z = r[3]

    vehicle_pose.orientation = q

    vehicle_pose.vel.x = ṙ[1]
    vehicle_pose.vel.y = ṙ[2]
    vehicle_pose.vel.z = ṙ[3]

    vehicle_pose.angular_vel.x = ω[1]
    vehicle_pose.angular_vel.y = ω[2]
    vehicle_pose.angular_vel.z = ω[3]

    flight_controller.ram_memory[:vehicle_pose] = vehicle_pose

    # # run the scheduler ------------------------------------------------
    ctrl_cmd, ref = scheduler(flight_controller)

    # set the control input
    c_index = 18

    int.u[c_index] = ctrl_cmd.motor_thrusts[1]
    int.u[c_index+1] = ctrl_cmd.motor_thrusts[2]
    int.u[c_index+2] = ctrl_cmd.motor_thrusts[3]
    int.u[c_index+3] = ctrl_cmd.motor_thrusts[4]


    # set the reference
    r_ref_id = 22
    int.u[r_ref_id] = ref.pos.x
    int.u[r_ref_id+1] = ref.pos.y
    int.u[r_ref_id+2] = ref.pos.z

    ṙ_ref_id = 25
    int.u[ṙ_ref_id] = ref.vel.x
    int.u[ṙ_ref_id+1] = ref.vel.y
    int.u[ṙ_ref_id+2] = ref.vel.z

    # q_ref_id = 28
    # int.u[q_ref_id] = ref.pos.x
    # int.u[q_ref_id+1] = ref.pos.y
    # int.u[q_ref_id+2] = ref.pos.z
    # int.u[q_ref_id+3] = ref.pos.z

    ω_ref_id = 32
    int.u[ω_ref_id] = ctrl_cmd.angular_vel.x
    int.u[ω_ref_id+1] = ctrl_cmd.angular_vel.x
    int.u[ω_ref_id+2] = ctrl_cmd.angular_vel.x
end
