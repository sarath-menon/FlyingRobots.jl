
# function position_controller(strategy::SimplePid_PosCtlr, computer, rate_hz)

#     g = 9.81
#     dt = 1 / rate_hz

#     trajectory_reference = computer.ram_memory[:trajectory_reference]
#     ctrl_cmd = computer.ram_memory[:ctrl_cmd]
#     vehicle_pose = computer.ram_memory[:vehicle_pose]

#     vehicle_params = computer.rom_memory.params[:vehicle]
#     m::Float64 = vehicle_params.mass

#     # joystick
#     js = computer.rom_memory.sensors.joystick
#     js_state = Joystick.get_joystick_state(js)
#     # Core.println(js_state)

#     x_pos_pid = computer.rom_memory.pid.x_pos
#     y_pos_pid = computer.rom_memory.pid.y_pos
#     z_pos_pid = computer.rom_memory.pid.z_pos

#     e_x::Float64 = trajectory_reference.pos.x - vehicle_pose.pos.x
#     e_y::Float64 = trajectory_reference.pos.y - vehicle_pose.pos.y
#     e_z::Float64 = trajectory_reference.pos.z - vehicle_pose.pos.z

#     ẍ_cmd = trajectory_reference.acc.x
#     ÿ_cmd = trajectory_reference.acc.y
#     z̈_cmd = trajectory_reference.acc.z

#     # x position controller
#     ctrl_cmd.orientation_euler.q = ẍ_cmd + pid_controller!(x_pos_pid; e=e_x, dt=dt, umin=-0.5, umax=0.5) / g

#     # y position controller
#     ctrl_cmd.orientation_euler.p = ÿ_cmd + -pid_controller!(y_pos_pid; e=e_y, dt=dt, umin=-0.5, umax=0.5) / g
# end


function position_controller(strategy::P_PosCtlr, computer, rate_hz)

    g = 9.81
    dt = 1 / rate_hz

    ref = computer.ram_memory[:trajectory_reference]
    vehicle_pose = computer.ram_memory[:vehicle_pose]

    # get task memory
    task_mem = computer.ram_memory[:task_mem]

    # PID dict
    pid = task_mem[:position_controller][:PID]

    e_x::Float64 = ref.pos.x - vehicle_pose.pos.x
    e_y::Float64 = ref.pos.y - vehicle_pose.pos.y
    e_z::Float64 = ref.pos.z - vehicle_pose.pos.z

    # max velocity, hardcoded for now
    vel_max = 10 #m/s

    # x position controller
    ref.vel.x = pid_controller!(pid[:x]; e=e_x, dt=dt, umin=-vel_max, umax=vel_max)

    # y position controller
    ref.vel.y = pid_controller!(pid[:y]; e=e_y, dt=dt, umin=-vel_max, umax=vel_max)

    # z position controller
    ref.vel.z = pid_controller!(pid[:z]; e=e_z, dt=dt, umin=-vel_max, umax=vel_max)
end

function velocity_controller(strategy::Pid_VelCtlr, computer, rate_hz)

    dt = 1 / rate_hz

    ref = computer.ram_memory[:trajectory_reference]
    vehicle_pose = computer.ram_memory[:vehicle_pose]

    # # joystick
    # js = computer.rom_memory.sensors.joystick
    # js_state = Joystick.get_joystick_state(js)
    # # Core.println(js_state)

    # get task memory
    task_mem = computer.ram_memory[:task_mem]

    # PID dict
    pid = task_mem[:velocity_controller][:PID]

    e_x::Float64 = ref.vel.x - vehicle_pose.vel.x
    e_y::Float64 = ref.vel.y - vehicle_pose.vel.y
    e_z::Float64 = ref.vel.z - vehicle_pose.vel.z

    # max acceleration, hardcoded for now
    acc_max = 10 #m/s^2

    # x velocity controller
    ref.acc.x = pid_controller!(pid[:x]; e=e_x, dt=dt, umin=-acc_max, umax=acc_max)

    # y velocity controller
    ref.acc.y = pid_controller!(pid[:y]; e=e_y, dt=dt, umin=-acc_max, umax=acc_max)

    # z velocity controller
    ref.acc.z = pid_controller!(pid[:z]; e=e_z, dt=dt, umin=-acc_max, umax=acc_max)
end


function acceleration_controller(strategy::Linear_AccCtlr, computer, rate_hz)

    g = 9.81
    dt = 1 / rate_hz

    ref = computer.ram_memory[:trajectory_reference]
    ctrl_cmd = computer.ram_memory[:ctrl_cmd]

    vehicle_params = computer.rom_memory.params[:vehicle]
    m::Float64 = vehicle_params.mass

    # max thrust, attitude, hardcoded for now
    f_max = 45 #N
    f_min = 7 #N

    p_max = 0.2 #rad
    q_max = 0.2 #rad

    # control laws
    f_cmd = m * (g + ref.acc.z)
    q_cmd = ref.acc.x / g
    p_cmd = -ref.acc.y / g

    # apply saturation
    f_cmd_saturated = clamp(f_cmd, f_min, f_max)
    q_cmd_saturated = clamp(q_cmd, -q_max, q_max)
    p_cmd_saturated = clamp(p_cmd, -p_max, p_max)

    # pitch controller
    ctrl_cmd.orientation_euler.q = q_cmd_saturated

    # roll controller
    ctrl_cmd.orientation_euler.p = p_cmd_saturated

    # z position controller
    ctrl_cmd.f_net.z = f_cmd_saturated
end

