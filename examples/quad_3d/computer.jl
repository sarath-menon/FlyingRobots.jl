
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

function reset_controllers!(computer::OnboardComputer)
    for ctrl in computer.rom_memory[:pid]
        FlyingRobots.reset!(ctrl)
    end
end

function update_controller_params(computer::OnboardComputer)

    pid = computer.rom_memory.pid
    controller = computer.rom_memory.params[:controller]

    update!(pid.x_pos, controller[:position][:pid_x])
    update!(pid.y_pos, controller[:position][:pid_y])
    update!(pid.z_pos, controller[:position][:pid_z])

    update!(pid.x_vel, controller[:velocity][:pid_x])
    update!(pid.y_vel, controller[:velocity][:pid_y])
    update!(pid.z_vel, controller[:velocity][:pid_z])

end

function initialize!(params_dict)

    vehicle_params = params_dict[:vehicle]

    ctrl_params = params_dict[:controller]

    # parameters

    # create pid objects
    x_pos_pid = PID(ctrl_params[:position][:pid_x])
    y_pos_pid = PID(ctrl_params[:position][:pid_y])
    z_pos_pid = PID(ctrl_params[:position][:pid_z])

    x_vel_pid = PID(ctrl_params[:velocity][:pid_x])
    y_vel_pid = PID(ctrl_params[:velocity][:pid_y])
    z_vel_pid = PID(ctrl_params[:velocity][:pid_z])

    roll_pid = PID(ctrl_params[:attitude][:pid_roll])
    pitch_pid = PID(ctrl_params[:attitude][:pid_pitch])

    # joystick
    js = Joystick.connect_joystick()

    # ROM memory

    pid = (; x_pos=x_pos_pid, y_pos=y_pos_pid, z_pos=z_pos_pid,
        x_vel=x_vel_pid, y_vel=y_vel_pid, z_vel=z_vel_pid,
        roll=roll_pid, pitch=pitch_pid)

    sensors = (; joystick=js)

    allocation_matrix = ctrl_params[:allocation_matrix]

    rom_memory = (; params=params_dict, pid=pid,
        allocation_matrix=allocation_matrix, sensors=sensors)

    # RAM memory
    ram_memory = Dict{Symbol,Any}()
    ram_memory[:ctrl_cmd] = CascadedPidCtrlCmd()
    ram_memory[:vehicle_pose] = Pose3d()
    ram_memory[:trajectory_reference] = TrajectoryReference()

    return (rom_memory, ram_memory)
end

function FlyingRobots.reset!(computer::OnboardComputer)
    reset_clock!(computer.main_clock)

    # reset controller internal vars (eg. pid integral sum)
    reset_controllers!(computer)

    # update controller gains
    update_controller_params(computer)

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

