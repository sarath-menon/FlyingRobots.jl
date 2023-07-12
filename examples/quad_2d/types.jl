export Quad2D, Quad2DState, Quad2DActuatorCmd

mutable struct Quad2DState1 <: FrRobotState
    y::Float64
    z::Float64
    θ::Float64
    ẏ::Float64
    ż::Float64
    θ̇::Float64
end
Quad2DState = Quad2DState1

# ------------------------------------------------

mutable struct Quad2DActuatorCmd1 <: FrRobotCtrlCmd
    left_motor_thrust::Float64
    right_motor_thrust::Float64
end

Quad2DActuatorCmd = Quad2DActuatorCmd1

# ------------------------------------------------

mutable struct Quad2DControlCmd2 <: FrRobotCtrlCmd
    body_thrust::Float64
    body_torque::Float64
end

Quad2DControlCmd = Quad2DControlCmd2

# ------------------------------------------------
struct Quad2D1 <: FrRobot
    nx::Int
    nu::Int

    state::Quad2DState
    params::NamedTuple
end

Quad2D = Quad2D1

# ------------------------------------------------

# ------------------------------------------------
