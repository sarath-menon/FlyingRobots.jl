

module Computer

using YAML
using Rotations
using StaticArrays

# include("vehicle.jl")
# include("controller_utilities.jl")
# include("tasks.jl")
# include("scheduler.jl")
include("types.jl")

export increment_clock_timer!


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

# return time elapsed since bootup in seconds
function get_elapsed_time(computer)
    count = computer.main_clock.count
    clock_speed = computer.rom_memory.params.vehicle.computer.clock_speed

    return count / clock_speed
end

end

