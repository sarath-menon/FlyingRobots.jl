
function control_allocator(strategy::SimpleClipping_ControlAllocator, computer, rate_hz)

    ctrl_cmd = computer.ram_memory[:ctrl_cmd]

    # get task memory
    mem = computer.ram_memory[:task_mem][:control_allocator]

    allocation_matrix = mem[:allocation_matrix]

    ctrl_cmd.motor_thrusts = allocation_matrix * @SVector Float64[ctrl_cmd.f_net.z; ctrl_cmd.τ.x; ctrl_cmd.τ.y; ctrl_cmd.τ.z]
end

function initialize_task!(strategy::SimpleClipping_ControlAllocator, computer::OnboardComputer)
    vehicle_params = params_dict[:vehicle]
    mem = computer.ram_memory[:task_mem][:control_allocator]

    l = vehicle_params[:arm_length]
    k_τ = vehicle_params[:actuators][:constants][:k_τ]

    mem[:allocation_matrix] = body_thrust_to_motor_thrust(; l=l, k_τ=k_τ)
end



