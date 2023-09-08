
mutable struct CascadedPidCtrlCmd5
    vel::Vec3d

    orientation_euler::EulerAngleZYX

    angular_vel::Vec3d

    f_net::Vec3d

    τ::Vec3d

    motor_thrusts::Vector{Float64}

    CascadedPidCtrlCmd5() = new(Vec3d(), EulerAngleZYX(), Vec3d(), Vec3d(), Vec3d(), [0, 0, 0, 0])
end
CascadedPidCtrlCmd = CascadedPidCtrlCmd5

mutable struct TrajectoryReference5
    pos::Vec3d

    vel::Vec3d

    acc::Vec3d

    TrajectoryReference5() = new(Vec3d(), Vec3d(), Vec3d())
end

TrajectoryReference = TrajectoryReference5

## For strategy pattern

abstract type TrajectoryGen end
abstract type PosController end
abstract type VelController end
abstract type AttitudeController end
abstract type ControlAllocator end

# trajectory generators
struct Circle_TrajectoryGen <: TrajectoryGen end

# position controllers
struct SimplePid_PosController <: PosController end
struct P_PosController <: PosController end
struct ConvexMpc_PosController <: PosController end

# velocity controllers
struct Pid_VelController <: VelController end

# acceleration controllers
struct Linear_AccController <: VelController end

# attitude controllers
struct SimplePid_AttitudeController <: AttitudeController end

struct SimpleClipping_ControlAllocator <: ControlAllocator end


function load_type_name_from_yaml(name::String)
    a = Symbol(name)
    return @eval $a()
end
