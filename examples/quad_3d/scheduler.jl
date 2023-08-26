

function scheduler(computer, vehicle_pose, ctrl_cmd)

    #increment the main clock
    increment_clock(computer.main_clock)

    # execute tasks defined in yaml file in given order 
    for task in computer.tasks
        # run task if it's time
        if computer.main_clock.count % task.rate_per_tick == 0
            task.func(computer, vehicle_pose, ctrl_cmd, task.rate)
        end
    end

    # run control allocator
    control_allocator(computer, vehicle_pose, ctrl_cmd)
end




