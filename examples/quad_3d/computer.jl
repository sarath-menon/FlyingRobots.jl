
using FlyingRobots.Computer: OnboardComputer, ComputerClock
using FlyingRobots.Computer: increment_clock!
import FlyingRobots.Computer: reset_clock!

function create_computer(name, params_dict)
    # SharedMemory objects ----------------------------------------------------

    # parameters
    vehicle_params = params_dict[:vehicle]

    # clock
    main_clock = ComputerClock(; speed=vehicle_params.computer.clock_speed)

    # initialization
    (rom_memory, ram_memory) = initialize!(params_dict)

    return OnboardComputer(name; main_clock=main_clock, ram_memory=ram_memory,
        rom_memory=rom_memory, tasks=vehicle_params.computer.tasks)
end


function initialize!(params_dict)

    vehicle_params = params_dict[:vehicle]

    # joystick
    js = Joystick.connect_joystick()

    # ROM memory
    sensors = (; joystick=js)

    rom_memory = (; params=params_dict, sensors=sensors)

    # RAM memory
    ram_memory = Dict{Symbol,Any}()
    ram_memory[:ctrl_cmd] = CascadedPidCtrlCmd()
    ram_memory[:vehicle_pose] = Pose3d()
    ram_memory[:trajectory_reference] = TrajectoryReference()

    initialize_task_stack!(ram_memory, params_dict)

    return (rom_memory, ram_memory)
end

function FlyingRobots.reset!(computer::OnboardComputer, params_dict)
    reset_clock!(computer.main_clock)

    initialize_task_stack!(computer, params_dict)

    # reset RAM
    computer.ram_memory[:ctrl_cmd] = CascadedPidCtrlCmd()
    computer.ram_memory[:vehicle_pose] = Pose3d()
    computer.ram_memory[:trajectory_reference] = TrajectoryReference()
end

function scheduler(computer)

    #increment the main clock
    increment_clock!(computer.main_clock)

    # execute tasks defined in yaml file in given order 
    for task in computer.tasks
        # run task if it's time
        if computer.main_clock.count % task.rate_per_tick == 0
            task.func(task.strategy, computer, task.rate)
        end
    end

    # return the  trajectory_reference and motor thrusts
    trajectory_reference = computer.ram_memory[:trajectory_reference]
    ctrl_cmd = computer.ram_memory[:ctrl_cmd]

    return ctrl_cmd.motor_thrusts, trajectory_reference
end

