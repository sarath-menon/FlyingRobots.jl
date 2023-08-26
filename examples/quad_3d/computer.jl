

module Computer

using YAML
using FlyingRobots
using Rotations

include("vehicle.jl")
include("controller_utilities.jl")
include("scheduler.jl")
include("tasks.jl")
include("computer_types.jl")
include("types.jl")

export scheduler, increment_clock_timer!, create_computer

function create_computer(name)

    folder_path = pwd() * "/examples/quad_3d"

    vehicle_params_path = "/parameters/vehicle.yml"
    ctrl_yaml_path = "/parameters/controller.yml"

    # SharedMemory objects ----------------------------------------------------

    # clock
    main_clock = ComputerClock1()

    # parameters
    vehicle_params = load_vehicle_params_computer(folder_path * vehicle_params_path)
    ctrl_yaml = load_controller_params(folder_path * ctrl_yaml_path, vehicle_params)

    # create pid objects
    x_pos_pid = PID(ctrl_yaml[:position_controller][:pid_x])
    y_pos_pid = PID(ctrl_yaml[:position_controller][:pid_y])
    z_pos_pid = PID(ctrl_yaml[:position_controller][:pid_z])

    roll_pid = PID(ctrl_yaml[:attitude_controller][:pid_roll])
    pitch_pid = PID(ctrl_yaml[:attitude_controller][:pid_pitch])

    # ROM memory
    params = (; vehicle=vehicle_params)
    pid = (; x_pos=x_pos_pid, y_pos=y_pos_pid, z_pos=z_pos_pid,
        roll=roll_pid, pitch=pitch_pid)

    allocation_matrix = ctrl_yaml[:allocation_matrix]

    rom_memory = (; params=params, pid=pid, allocation_matrix=allocation_matrix)

    # RAM memory
    ram_memory = Dict()
    ram_memory[:ctrl_cmd] = CascadedPidCtrlCmd()
    ram_memory[:vehicle_pose] = Pose3d()
    ram_memory[:trajectory_reference] = TrajectoryReference()



    return OnboardComputer(name; main_clock=main_clock, ram_memory=ram_memory,
        rom_memory=rom_memory, tasks=vehicle_params.computer.tasks)
end


function increment_clock(clock)
    clock.count += 1
end

function reset_clock(clock)
    clock.count = 0
end

function reset_memory()
    pid_reset(x_pos_pid)
    pid_reset(y_pos_pid)
    pid_reset(z_pos_pid)

    pid_reset(roll_pid)
    pid_reset(pitch_pid)
end

function full_reset()
    reset_clock(main_clock)
    reset_memory()
end

function load_vehicle_params_computer(path::String)
    vehicle_yaml = YAML.load_file(path; dicttype=Dict{Symbol,Any})

    # # set tasks per ticks for each computer task
    for task in vehicle_yaml[:computer][:tasks]
        clock_speed = vehicle_yaml[:computer][:clock_speed]
        task_rate_hz = task[:rate]
        task[:rate_per_tick] = Int(clock_speed / task_rate_hz)

        task[:func] = getfield(Main.Computer, Symbol(task[:name]))
    end

    vehicle_params = recursive_dict_to_namedtuple(vehicle_yaml)

    return vehicle_params
end

end

