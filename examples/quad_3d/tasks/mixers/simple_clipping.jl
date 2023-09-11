
function control_allocator(strategy::SimpleClipping_ControlAllocator, computer, rate_hz)

    ctrl_cmd = computer.ram_memory[:ctrl_cmd]

    # get task memory
    task_mem = computer.ram_memory[:task_mem]

    allocation_matrix = task_mem[:control_allocator][:body_thrust_to_motor_thrust][:allocation_matrix]

    ctrl_cmd.motor_thrusts = allocation_matrix * @SVector Float64[ctrl_cmd.f_net.z; ctrl_cmd.τ.x; ctrl_cmd.τ.y; ctrl_cmd.τ.z]
end