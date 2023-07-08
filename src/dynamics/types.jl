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
struct SimParams2
    nx::Int8
    nu::Int8
    ny::Int8
    Ts::Float64

end
SimParams = SimParams2
#--------------------------------------------------------------------------
struct SafetyBox2
    x_max::Float64
    y_max::Float64
    z_max::Float64

    x_min::Float64
    y_min::Float64
    z_min::Float64

end
SafetyBox = SafetyBox2
