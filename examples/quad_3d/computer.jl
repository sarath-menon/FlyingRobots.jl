
using FlyingRobots.Computer: OnboardComputer, ComputerClock
using FlyingRobots.Computer: increment_clock!
import FlyingRobots.Computer: reset_clock!

function create_computer(name)
    # SharedMemory objects ----------------------------------------------------

    # parameters
    vehicle_params = load_vehicle_params(folder_path * vehicle_params_path)
    ctrl_yaml = load_controller_params(folder_path * ctrl_yaml_path, vehicle_params)

    # clock
    main_clock = ComputerClock(; speed=vehicle_params.computer.clock_speed)

    # create pid objects
    x_pos_pid = PID(ctrl_yaml[:position_controller][:pid_x])
    y_pos_pid = PID(ctrl_yaml[:position_controller][:pid_y])
    z_pos_pid = PID(ctrl_yaml[:position_controller][:pid_z])

    x_vel_pid = PID(ctrl_yaml[:velocity_controller][:pid_x])
    y_vel_pid = PID(ctrl_yaml[:velocity_controller][:pid_y])
    z_vel_pid = PID(ctrl_yaml[:velocity_controller][:pid_z])

    roll_pid = PID(ctrl_yaml[:attitude_controller][:pid_roll])
    pitch_pid = PID(ctrl_yaml[:attitude_controller][:pid_pitch])

    # joystick
    js = Joystick.connect_joystick()

    # ROM memory
    params = (; vehicle=vehicle_params)
    pid = (; x_pos=x_pos_pid, y_pos=y_pos_pid, z_pos=z_pos_pid,
        x_vel=x_vel_pid, y_vel=y_vel_pid, z_vel=z_vel_pid,
        roll=roll_pid, pitch=pitch_pid)

    funcs =
        sensors = (; joystick=js)

    allocation_matrix = ctrl_yaml[:allocation_matrix]

    rom_memory = (; params=params, pid=pid,
        allocation_matrix=allocation_matrix, sensors=sensors)

    # RAM memory
    ram_memory = Dict{Symbol,Any}()
    ram_memory[:ctrl_cmd] = CascadedPidCtrlCmd()
    ram_memory[:vehicle_pose] = Pose3d()
    ram_memory[:trajectory_reference] = TrajectoryReference()

    return OnboardComputer(name; main_clock=main_clock, ram_memory=ram_memory,
        rom_memory=rom_memory, tasks=vehicle_params.computer.tasks)
end

function reset_controllers!(computer::OnboardComputer)
    for ctrl in computer.rom_memory[:pid]
        FlyingRobots.reset!(ctrl)
    end
end

function FlyingRobots.reset!(computer::OnboardComputer)
    reset_clock!(computer.main_clock)
    reset_controllers!(computer)

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

