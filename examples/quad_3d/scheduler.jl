

function scheduler(clock, vehicle_pose, ctrl_cmd, vehicle_params)

    # tasks defined in yaml file in given order 

    for task in vehicle_params.computer.tasks
        # run task if it's time
        if clock % task.rate_per_tick == 0
            task.func(vehicle_pose, ctrl_cmd, vehicle_params, task.rate)
        end
    end

    # run control allocator
    control_allocator(vehicle_pose, ctrl_cmd, vehicle_params)
end




