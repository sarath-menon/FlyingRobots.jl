
mutable struct CascadedPidCtrlCmd11
    x::Float64
    y::Float64
    z::Float64

    p::Float64
    q::Float64
    r::Float64

    ẋ::Float64
    ẏ::Float64
    ż::Float64

    ṗ::Float64
    q̇::Float64
    ṙ::Float64

    f_net::Float64

    τ_x::Float64
    τ_y::Float64
    τ_z::Float64

    motor_thrusts::Vector{Float64}

    CascadedPidCtrlCmd11() = new(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, [0, 0, 0, 0])
end
CascadedPidCtrlCmd = CascadedPidCtrlCmd11

# struct Vec3d2 <: FieldVector{3,Float64}
#     x::Float64
#     y::Float64
#     z::Float64

#     Vec3d2() = new(0, 0, 0)
#     Vec3d2(x, y, z) = new(x, y, z)
#     Vec3d2(v::AbstractArray) = new(v[1], v[2], v[3])
#     Vec3d2(v::Tuple) = new(v[1], v[2], v[3])
# end

# mutable struct Vec3d3
#     x::Float64
#     y::Float64
#     z::Float64

#     Vec3d3() = new(0, 0, 0)
#     Vec3d3(x, y, z) = new(x, y, z)
#     Vec3d3(v::AbstractArray) = new(v[1], v[2], v[3])
#     Vec3d3(v::Tuple) = new(v[1], v[2], v[3])
# end


# Vec3d = Vec3d3

# mutable struct Pose3d7
#     pos::Vec3d
#     vel::Vec3d
#     orientation::QuatRotation
#     angular_vel::Vec3d

#     Pose3d7() = new(Vec3d(), Vec3d(), one(QuatRotation), Vec3d())
#     Pose3d7(pos::Vec3d, vel::Vec3d, q::QuatRotation, angular_vel::Vec3d) = new(pos, vel, q, angular_vel)
# end

# Pose3d = Pose3d7

