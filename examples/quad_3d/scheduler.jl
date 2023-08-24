

function scheduler(clock, vehicle_pose, ctrl_cmd, vehicle_params)

    # tasks defined in yaml file in given order 

    for task in vehicle_params.computer.tasks
        # get function from task name
        func = getfield(Main, Symbol(task.name))

        # run task it it's time
        if clock % task.rate_per_tick == 0
            func(vehicle_pose, ctrl_cmd, vehicle_params, task.rate)
        end
    end

    # run control allocator
    control_allocator(vehicle_pose, ctrl_cmd, vehicle_params)
end



