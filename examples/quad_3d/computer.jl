

module Computer

using YAML
using FlyingRobots

include("vehicle.jl")
include("controller_utilities.jl")
include("scheduler.jl")
include("tasks.jl")
include("computer_types.jl")

export scheduler, increment_clock_timer!

folder_path = pwd() * "/examples/quad_3d"

vehicle_params_path = "/parameters/vehicle.yml"
ctrl_yaml_path = "/parameters/controller.yml"

# SharedMemory objects ----------------------------------------------------

# clock
main_clock = ComputerClock1()

#parameters
vehicle_params = load_vehicle_params(folder_path * vehicle_params_path)
ctrl_yaml = load_controller_params(folder_path * ctrl_yaml_path)

# pid controllers
x_pos_pid = PID(ctrl_yaml[:position_controller][:pid_x])
y_pos_pid = PID(ctrl_yaml[:position_controller][:pid_y])
z_pos_pid = PID(ctrl_yaml[:position_controller][:pid_z])

roll_pid = PID(ctrl_yaml[:attitude_controller][:pid_roll])
pitch_pid = PID(ctrl_yaml[:attitude_controller][:pid_pitch])

memory = Dict()

memory[:x_pos_pid] = x_pos_pid
memory[:y_pos_pid] = y_pos_pid
memory[:z_pos_pid] = z_pos_pid

memory[:roll_pid] = roll_pid
memory[:pitch_pid] = pitch_pid

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

end

