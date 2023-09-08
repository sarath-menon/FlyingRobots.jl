include("simple_clipping.jl")


function control_allocator(strategy::SimpleClipping_ControlAllocator, computer, rate_hz)

    ctrl_cmd = computer.ram_memory[:ctrl_cmd]

    allocation_matrix::Matrix{Float64} = computer.rom_memory.allocation_matrix

    ctrl_cmd.motor_thrusts = allocation_matrix * @SVector Float64[ctrl_cmd.f_net.z; ctrl_cmd.τ.x; ctrl_cmd.τ.y; ctrl_cmd.τ.z]

    # return motor_thrusts
end