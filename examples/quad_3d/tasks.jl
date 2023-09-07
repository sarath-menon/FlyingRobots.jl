
using FlyingRobots.Computer: get_elapsed_time

function position_controller(strategy::SimplePid_PosController, computer, rate_hz)

    g = 9.81
    dt = 1 / rate_hz

    trajectory_reference = computer.ram_memory[:trajectory_reference]
    ctrl_cmd = computer.ram_memory[:ctrl_cmd]
    vehicle_pose = computer.ram_memory[:vehicle_pose]

    vehicle_params = computer.rom_memory.params.vehicle
    m::Float64 = vehicle_params.mass

    # joystick
    js = computer.rom_memory.sensors.joystick
    js_state = Joystick.get_joystick_state(js)
    # Core.println(js_state)

    x_pos_pid = computer.rom_memory.pid.x_pos
    y_pos_pid = computer.rom_memory.pid.y_pos
    z_pos_pid = computer.rom_memory.pid.z_pos

    e_x::Float64 = trajectory_reference.pos.x - vehicle_pose.pos.x
    e_y::Float64 = trajectory_reference.pos.y - vehicle_pose.pos.y
    e_z::Float64 = trajectory_reference.pos.z - vehicle_pose.pos.z

    ẍ_cmd = trajectory_reference.acc.x
    ÿ_cmd = trajectory_reference.acc.y
    z̈_cmd = trajectory_reference.acc.z

    # x position controller
    ctrl_cmd.orientation_euler.q = ẍ_cmd + pid_controller!(x_pos_pid; e=e_x, dt=dt, umin=-0.5, umax=0.5) / g

    # y position controller
    ctrl_cmd.orientation_euler.p = ÿ_cmd + -pid_controller!(y_pos_pid; e=e_y, dt=dt, umin=-0.5, umax=0.5) / g

    # z position controller
    ctrl_cmd.f_net.z = m * (g + z̈_cmd + pid_controller!(z_pos_pid; e=e_z, dt=dt, umin=-4.5, umax=40.0))
end


function attitude_controller(strategy::SimplePid_AttitudeController, computer, rate_hz)

    dt = 1 / rate_hz

    ctrl_cmd = computer.ram_memory[:ctrl_cmd]
    vehicle_pose = computer.ram_memory[:vehicle_pose]

    roll_pid = computer.rom_memory.pid.roll
    pitch_pid = computer.rom_memory.pid.pitch

    # convert quaternion attitude representation to euler angles 
    r::Float64, q::Float64, p::Float64 = Rotations.params(RotZYX(vehicle_pose.orientation))

    e_p::Float64 = ctrl_cmd.orientation_euler.p - p
    e_q::Float64 = ctrl_cmd.orientation_euler.q - q

    ctrl_cmd.τ.x = pid_controller!(roll_pid; e=e_p, dt=dt, umin=-25, umax=25)
    ctrl_cmd.τ.y = pid_controller!(pitch_pid; e=e_q, dt=dt, umin=-25, umax=25)
    ctrl_cmd.τ.z = 0
end


function control_allocator(strategy::SimpleClipping_ControlAllocator, computer, rate_hz)

    ctrl_cmd = computer.ram_memory[:ctrl_cmd]

    allocation_matrix::Matrix{Float64} = computer.rom_memory.allocation_matrix

    ctrl_cmd.motor_thrusts = allocation_matrix * @SVector Float64[ctrl_cmd.f_net.z; ctrl_cmd.τ.x; ctrl_cmd.τ.y; ctrl_cmd.τ.z]

    # return motor_thrusts
end


function reference_generator(strategy::Circle_TrajectoryGen, computer::OnboardComputer, rate_hz::Int)

    t = get_elapsed_time(flight_controller)

    # get trajectory reference command
    ref = get_circle_trajectory(t)

    # set the trajectory reference
    trajectory_reference = computer.ram_memory[:trajectory_reference]

    trajectory_reference.pos.x = ref[1]
    trajectory_reference.pos.y = ref[2]
    trajectory_reference.pos.z = ref[3]

end

function get_circle_trajectory(t)

    r = 0.5    # circle radius 
    ω = 0.5    # angular velocity

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
