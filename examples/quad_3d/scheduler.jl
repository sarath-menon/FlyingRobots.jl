

function scheduler(vehicle_pose, ctrl_cmd)

    # tasks defined in yaml file in given order 

    for task in vehicle_params.computer.tasks
        # run task if it's time
        if main_clock.count % task.rate_per_tick == 0
            task.func(vehicle_pose, ctrl_cmd, vehicle_params, task.rate)
        end
    end

    # run control allocator
    control_allocator(vehicle_pose, ctrl_cmd, vehicle_params)
end




