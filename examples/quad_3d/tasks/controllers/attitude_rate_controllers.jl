function attitude_rate_controller(strategy::SimplePid_AttRateCtlr, computer, rate_hz)

    dt = 1 / rate_hz

    ctrl_cmd = computer.ram_memory[:ctrl_cmd]
    vehicle_pose = computer.ram_memory[:vehicle_pose]

    # get task memory
    task_mem = computer.ram_memory[:task_mem]

    # PID dict
    pid = task_mem[:attitude_rate_controller][:PID]

    # convert quaternion attitude representation to euler angles 
    ω = vehicle_pose.angular_vel
    ω_cmd = ctrl_cmd.angular_vel

    e_x::Float64 = ω_cmd.x - ω.x
    e_y::Float64 = ω_cmd.y - ω.y
    e_z::Float64 = ω_cmd.z - ω.z

    ctrl_cmd.τ.x = pid_controller!(pid[:x]; e=e_x, dt=dt, umin=-25, umax=25)
    ctrl_cmd.τ.y = pid_controller!(pid[:y]; e=e_y, dt=dt, umin=-25, umax=25)
    ctrl_cmd.τ.z = 0
end
