
mutable struct CascadedPidCtrlCmd6
    orientation_euler::EulerAngleZYX

    angular_vel::Vec3d

    f_net::Vec3d

    τ::Vec3d

    motor_thrusts::Vector{Float64}

    CascadedPidCtrlCmd6() = new(EulerAngleZYX(), Vec3d(), Vec3d(), Vec3d(), [0, 0, 0, 0])
end
CascadedPidCtrlCmd = CascadedPidCtrlCmd6


mutable struct TrajectoryReference5
    pos::Vec3d

    vel::Vec3d

    acc::Vec3d

    TrajectoryReference5() = new(Vec3d(), Vec3d(), Vec3d())
end

TrajectoryReference = TrajectoryReference5

## For strategy pattern

abstract type TrajGen end
abstract type PosCtlr end
abstract type VelCtlr end
abstract type AccCtlr end

abstract type AttCtlr end
abstract type AttRateCtlr end

abstract type CtrlAlloc end

# trajectory generators
struct Circle_TrajGen <: TrajGen end

# position controllers
struct SimplePid_PosCtlr <: PosCtlr end
struct P_PosCtlr <: PosCtlr end
struct ConvexMpc_PosCtlr <: PosCtlr end

# velocity controllers
struct Pid_VelCtlr <: VelCtlr end

# acceleration controllers
struct Linear_AccCtlr <: VelCtlr end

# attitude controllers
struct SimplePid_AttCtlr <: AttCtlr end
struct SimpleP_AttCtlr <: AttCtlr end
struct QuatP_AttCtlr <: AttCtlr end

# attitude rate controllers
struct SimplePid_AttRateCtlr <: AttRateCtlr end

# control allocators
struct SimpleClipping_CtrlAlloc <: CtrlAlloc end


function load_type_name_from_yaml(name::String)
    a = Symbol(name)
    return @eval $a()
end
