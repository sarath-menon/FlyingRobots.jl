

# export BLDCMotor, Quad2d, FrModel, SafetyBox
# export Pose2D
# export Trajectory, CircleTrajectory
# export LQRController
# export Quad2dPlot

# export FrRobotState, FrCtrlCmd, FrRobotDynamics, FrRobot
# export FrCtrlCmd, FrActuatorCmd
# export FrDigitalController
# export RigidBody2D, RigidBody3D
export Vec3d, Pose3d

abstract type FrSys end

abstract type FrContinuousSys <: FrSys end
abstract type FrDiscreteSys <: FrSys end
abstract type FrHybridSys <: FrSys end

abstract type FrRobot <: FrHybridSys end

abstract type FrDigitalController <: FrDiscreteSys end

abstract type FrRobotDynamics <: FrContinuousSys end

abstract type FrState end
abstract type FrRobotState <: FrState end

abstract type FrCtrlCmd end
abstract type FrActuatorCmd end

abstract type RigidBody2D <: FieldVector{6,Float64} end
abstract type RigidBody3D <: FieldVector{12,Float64} end

#--------------------------------------------------------------------------

struct BLDCMotor1
    thrust_min::Float64
    thrust_max::Float64
end

BLDCMotor = BLDCMotor1

#--------------------------------------------------------------------------

struct Quad2d2
    m::Float64
    L::Float64
    I_xx::Float64

    motor_left::BLDCMotor
    motor_right::BLDCMotor
end

Quad2d = Quad2d2

#--------------------------------------------------------------------------
mutable struct Pose2D1
    y::Float64
    z::Float64
    θ::Float64
    ẏ::Float64
    ż::Float64
    θ̇::Float64
end

Pose2D = Pose2D1
#--------------------------------------------------------------------------
struct FrModel2
    nx::Int
    nu::Int
    ny::Int
    Ts::Float64

end
FrModel = FrModel2
#--------------------------------------------------------------------------
struct SafetyBox5
    x_low::Float64
    x_high::Float64

    y_low::Float64
    y_high::Float64

    z_low::Float64
    z_high::Float64
end
SafetyBox = SafetyBox5

#------------------------------------------------------------------
abstract type Trajectory end;

struct CircleTrajectory1 <: Trajectory
    r::Float64
    ω::Float64
    y₀::Float64
    z₀::Float64
end

CircleTrajectory = CircleTrajectory1


#------------------------------------------------------------------

mutable struct Vec3d3
    x::Float64
    y::Float64
    z::Float64

    Vec3d3() = new(0, 0, 0)
    Vec3d3(x, y, z) = new(x, y, z)
    Vec3d3(v::AbstractArray) = new(v[1], v[2], v[3])
    Vec3d3(v::Tuple) = new(v[1], v[2], v[3])
end


Vec3d = Vec3d3

mutable struct Pose3d7
    pos::Vec3d
    vel::Vec3d
    orientation::QuatRotation
    angular_vel::Vec3d

    Pose3d7() = new(Vec3d(), Vec3d(), one(QuatRotation), Vec3d())
    Pose3d7(pos::Vec3d, vel::Vec3d, q::QuatRotation, angular_vel::Vec3d) = new(pos, vel, q, angular_vel)
end

Pose3d = Pose3d7

