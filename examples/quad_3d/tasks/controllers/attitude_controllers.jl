

function attitude_controller(strategy::SimplePid_AttitudeController, computer, rate_hz)

    dt = 1 / rate_hz

    ctrl_cmd = computer.ram_memory[:ctrl_cmd]
    vehicle_pose = computer.ram_memory[:vehicle_pose]

    # get task memory
    task_mem = computer.ram_memory[:task_mem]

    # PID dict
    pid = task_mem[:attitude_controller][:PID]

    # convert quaternion attitude representation to euler angles 
    r::Float64, q::Float64, p::Float64 = Rotations.params(RotZYX(vehicle_pose.orientation))

    e_p::Float64 = ctrl_cmd.orientation_euler.p - p
    e_q::Float64 = ctrl_cmd.orientation_euler.q - q

    ctrl_cmd.τ.x = pid_controller!(pid[:roll]; e=e_p, dt=dt, umin=-25, umax=25)
    ctrl_cmd.τ.y = pid_controller!(pid[:pitch]; e=e_q, dt=dt, umin=-25, umax=25)
    ctrl_cmd.τ.z = 0
end

