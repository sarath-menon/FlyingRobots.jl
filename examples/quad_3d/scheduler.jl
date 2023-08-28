

function scheduler(computer)

    Core.println("Waiting to be notified by the simulator")

    while true

        # wait for notification from sim
        lock(Main.condition)
        t = wait(Main.condition)


        Core.println("notified")

        #increment the main clock
        increment_clock(computer.main_clock)

        # get trajectory reference command
        R = reference_generator(t)

        # set the trajectory reference
        trajectory_reference = computer.ram_memory[:trajectory_reference]

        trajectory_reference.pos.x = R[1]
        trajectory_reference.pos.y = R[2]
        trajectory_reference.pos.z = R[3]

        # execute tasks defined in yaml file in given order 
        for task in computer.tasks
            # run task if it's time
            if computer.main_clock.count % task.rate_per_tick == 0
                task.func(computer, task.rate)
            end
        end

        # run control allocator
        motor_thrusts = control_allocator(computer)

        # put data in channel
        Core.println("Putting data in the channel")
        put!(Main.c1, motor_thrusts)

        # notify sim that one compute cycle is done
        lock(Main.condition)
        notify(Main.condition)
        # unlock(condition)

        # return motor_thrusts
    end
end


