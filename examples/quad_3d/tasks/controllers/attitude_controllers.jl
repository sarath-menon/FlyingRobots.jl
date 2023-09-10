

function attitude_controller(strategy::SimplePid_AttitudeController, computer, rate_hz)

    dt = 1 / rate_hz

    ctrl_cmd = computer.ram_memory[:ctrl_cmd]
    vehicle_pose = computer.ram_memory[:vehicle_pose]

    # roll_pid = computer.rom_memory.pid.roll
    # pitch_pid = computer.rom_memory.pid.pitch

    roll_pid = computer.ram_memory[:attitude_pid][:roll]
    pitch_pid = computer.ram_memory[:attitude_pid][:pitch]

    # convert quaternion attitude representation to euler angles 
    r::Float64, q::Float64, p::Float64 = Rotations.params(RotZYX(vehicle_pose.orientation))

    e_p::Float64 = ctrl_cmd.orientation_euler.p - p
    e_q::Float64 = ctrl_cmd.orientation_euler.q - q

    ctrl_cmd.τ.x = pid_controller!(roll_pid; e=e_p, dt=dt, umin=-25, umax=25)
    ctrl_cmd.τ.y = pid_controller!(pitch_pid; e=e_q, dt=dt, umin=-25, umax=25)
    ctrl_cmd.τ.z = 0
end

function initialize!(strategy::SimplePid_AttitudeController, computer)

    params_dict = computer.rom_memory.params
    ctrl_params = params_dict[:controller]

    roll_pid = PID(ctrl_params[:attitude][:pid_roll])
    pitch_pid = PID(ctrl_params[:attitude][:pid_pitch])

    attitude_pid = (roll=roll_pid, pitch=pitch_pid)

    computer.ram_memory[:attitude_pid] = attitude_pid
end