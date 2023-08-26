
mutable struct CascadedPidCtrlCmd5
    vel::Vec3d

    orientation_euler::EulerAngleZYX

    angular_vel::Vec3d

    f_net::Vec3d

    τ::Vec3d

    motor_thrusts::Vector{Float64}

    CascadedPidCtrlCmd5() = new(Vec3d(), EulerAngleZYX(), Vec3d(), Vec3d(), Vec3d(), [0, 0, 0, 0])
end
CascadedPidCtrlCmd = CascadedPidCtrlCmd4

mutable struct TrajectoryReference3
    pos::Vec3d

    vel::Vec3d

    acc::Vec3d

    TrajectoryReference3() = new(Vec3d(), Vec3d(), Vec3d())
end

TrajectoryReference = TrajectoryReference3