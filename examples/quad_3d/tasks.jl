


function position_controller(computer, vehicle_pose, ctrl_cmd, rate_hz)

    g = 9.81
    dt = 1 / rate_hz

    trajectory_reference = computer.ram_memory[:trajectory_reference]

    vehicle_params = computer.rom_memory.params.vehicle
    m = vehicle_params.mass

    x_pos_pid = computer.rom_memory.pid.x_pos
    y_pos_pid = computer.rom_memory.pid.y_pos
    z_pos_pid = computer.rom_memory.pid.z_pos

    e_x = trajectory_reference.pos.x - vehicle_pose.pos.x
    e_y = trajectory_reference.pos.y - vehicle_pose.pos.y
    e_z = trajectory_reference.pos.z - vehicle_pose.pos.z

    # x position controller
    ctrl_cmd.orientation_euler.q = pid_controller(x_pos_pid; e=e_x, dt=dt, umin=-0.5, umax=0.5) / g

    # y position controller
    ctrl_cmd.orientation_euler.p = -pid_controller(y_pos_pid; e=e_y, dt=dt, umin=-0.5, umax=0.5) / g

    # z position controller
    ctrl_cmd.f_net.z = m * g + pid_controller(z_pos_pid; e=e_z, dt=dt, umin=-4.5, umax=40.0)
end


function attitude_controller(computer, vehicle_pose, ctrl_cmd, rate_hz)

    dt = 1 / rate_hz

    roll_pid = computer.rom_memory.pid.roll
    pitch_pid = computer.rom_memory.pid.pitch

    # convert quaternion attitude representation to euler angles 
    r, q, p = Rotations.params(RotZYX(vehicle_pose.orientation))

    e_p = ctrl_cmd.orientation_euler.p - p
    e_q = ctrl_cmd.orientation_euler.q - q

    ctrl_cmd.τ.x = pid_controller(roll_pid; e=e_p, dt=dt, umin=-25, umax=25)
    ctrl_cmd.τ.y = pid_controller(pitch_pid; e=e_q, dt=dt, umin=-25, umax=25)
    ctrl_cmd.τ.z = 0
end


function control_allocator(computer, vehicle_pose, ctrl_cmd)

    vehicle_params = computer.rom_memory.params.vehicle
    allocation_matrix = computer.rom_memory.allocation_matrix

    ctrl_cmd.motor_thrusts = allocation_matrix * [ctrl_cmd.f_net.z; ctrl_cmd.τ.x; ctrl_cmd.τ.y; ctrl_cmd.τ.z]
end

# function reset_pid_controllers()
#     pid_reset(x_pos_pid)
#     pid_reset(y_pos_pid)
#     pid_reset(z_pos_pid)

#     pid_reset(roll_pid)
#     pid_reset(pitch_pid)
# end
