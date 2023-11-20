
using FlyingRobots.Computer: get_elapsed_time

function reference_generator(strategy::Circle_TrajGen, computer::OnboardComputer, rate_hz::Int)

    t = get_elapsed_time(computer)

    # get trajectory reference command
    ref = get_circle_trajectory(t)

    # to set the trajectory reference
    ref_cmd = computer.ram_memory[:trajectory_reference]

    ref_cmd.pos.x = ref[1]
    ref_cmd.pos.y = ref[2]
    ref_cmd.pos.z = ref[3]

end

function get_circle_trajectory(t)

    r = 10.0    # circle radius 
    ω = 0.3    # angular velocity

    z_0 = 1

    # circular trajectory 
    x_ref = r * cos(ω * t)
    y_ref = r * sin(ω * t)
    z_ref = 1.0

    # x_ref = 0.0
    # y_ref = 0.0
    # z_ref = 1.0

    return [x_ref, y_ref, z_ref]
end
