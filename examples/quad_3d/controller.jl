

function position_controller(vehicle_pose, ctrl_cmd, vehicle_params, rate_hz)

    m = vehicle_params.mass
    g = 9.81
    dt = 1 / rate_hz

    e_x = ctrl_cmd.x - vehicle_pose.pos.x
    e_y = ctrl_cmd.y - vehicle_pose.pos.y
    e_z = ctrl_cmd.z - vehicle_pose.pos.z

    # x position controller
    ctrl_cmd.q = pid_controller(x_pos_pid; e=e_x, dt=dt, umin=-0.5, umax=0.5) / g

    # y position controller
    ctrl_cmd.p = -pid_controller(y_pos_pid; e=e_y, dt=dt, umin=-0.5, umax=0.5) / g

    # z position controller
    ctrl_cmd.f_net = m * g + pid_controller(z_pos_pid; e=e_z, dt=dt, umin=-4.5, umax=40.0)
end


function attitude_controller(vehicle_pose, ctrl_cmd, vehicle_params, rate_hz)

    dt = 1 / rate_hz

    # convert quaternion attitude representation to euler angles 
    r, q, p = Rotations.params(RotZYX(vehicle_pose.orientation))

    e_p = ctrl_cmd.p - p
    e_q = ctrl_cmd.q - q

    ctrl_cmd.τ_x = pid_controller(roll_pid; e=e_p, dt=dt, umin=-25, umax=25)
    ctrl_cmd.τ_y = pid_controller(pitch_pid; e=e_q, dt=dt, umin=-25, umax=25)
    ctrl_cmd.τ_z = 0
end


function control_allocator(vehicle_pose, ctrl_cmd, vehicle_params)

    ctrl_cmd.motor_thrusts = ctrl_yaml[:allocation_matrix] * [ctrl_cmd.f_net; ctrl_cmd.τ_x; ctrl_cmd.τ_y; ctrl_cmd.τ_z]
end

function reset_pid_controllers()
    pid_reset(x_pos_pid)
    pid_reset(y_pos_pid)
    pid_reset(z_pos_pid)

    pid_reset(roll_pid)
    pid_reset(pitch_pid)
end
