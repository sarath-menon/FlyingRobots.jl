export Vec3d, Pose3d, EulerAngleZYX

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



mutable struct EulerAngleZYX2
    p::Float64
    q::Float64
    r::Float64

    EulerAngleZYX2() = new(0, 0, 0)
end
EulerAngleZYX = EulerAngleZYX2

