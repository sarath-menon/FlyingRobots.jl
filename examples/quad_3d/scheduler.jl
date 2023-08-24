function scheduler(clock, vehicle_pose, ctrl_cmd, vehicle_params)

    # if clock % tasks_per_ticks[:position_controller] == 0
    #     position_controller(vehicle_pose, ctrl_cmd, vehicle_params)
    #     # @show clock
    # end

    # if clock % tasks_per_ticks[:attitude_controller] == 0
    #     attitude_controller(vehicle_pose, ctrl_cmd, vehicle_params)
    #     # @show clock
    # end

    for task in vehicle_params.computer.tasks

        func = getfield(Main, Symbol(task.name))

        if clock % task.rate_per_tick == 0
            func(vehicle_pose, ctrl_cmd, vehicle_params, task.rate)
        end
    end

    control_allocator(vehicle_pose, ctrl_cmd, vehicle_params)
end

function get_ticks_per_task(task_rates)

    tasks_per_ticks = Dict()

    for key in keys(task_rates)

        rate = task_rates[key]
        tasks_per_ticks[key] = Int(100 / rate)
    end

    return tasks_per_ticks
end


