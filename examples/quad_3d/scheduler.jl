

function scheduler(computer)

    #increment the main clock
    increment_clock(computer.main_clock)

    # execute tasks defined in yaml file in given order 
    for task in computer.tasks
        # run task if it's time
        if computer.main_clock.count % task.rate_per_tick == 0
            task.func(computer, task.rate)
        end
    end

    # run control allocator
    motor_thrusts = control_allocator(computer)

    return motor_thrusts
end




