include("constants.jl")

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

struct LQRController1
    K::SMatrix
end

LQRController = LQRController1


#------------------------------------------------------------------
abstract type Trajectory end

struct CircleTrajectory1 <: Trajectory
    r::Float64
    ω::Float64
    y₀::Float64
    z₀::Float64
end

CircleTrajectory = CircleTrajectory1


#------------------------------------------------------------------
