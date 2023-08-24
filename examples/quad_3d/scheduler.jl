function scheduler(clock, vehicle_pose, ctrl_cmd, vehicle_params)

    if clock % tasks_per_ticks[:pos_ctrl_loop] == 0
        position_controller(vehicle_pose, ctrl_cmd, vehicle_params)
        # @show clock
    end
end

function get_ticks_per_task(task_rates)

    tasks_per_ticks = Dict()

    for key in keys(task_rates)

        rate = task_rates[key]
        tasks_per_ticks[key] = Int(100 / rate)
    end

    return tasks_per_ticks
end


function position_controller(vehicle_pose, ctrl_cmd, vehicle_params)

    m = vehicle_params.mass
    g = 9.81

    e_x = ctrl_cmd.x - vehicle_pose.pos.x
    e_y = ctrl_cmd.y - vehicle_pose.pos.y
    e_z = ctrl_cmd.z - vehicle_pose.pos.z

    # x position controller
    ctrl_cmd.q = pid_controller(x_pos_pid; e=e_x, umin=-0.5, umax=0.5) / g

    # y position controller
    ctrl_cmd.p = -pid_controller(y_pos_pid; e=e_y, umin=-0.5, umax=0.5) / g

    # z position controller
    ctrl_cmd.f_net = m * g + pid_controller(z_pos_pid; e=e_z, umin=-4.5, umax=40.0)

    @show ctrl_cmd.f_net

end