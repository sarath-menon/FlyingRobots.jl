export Quad2D, Quad2DState, Quad2DActuatorCmd
export Quad2DController, Quad2DControlCmd

mutable struct Quad2DState2 <: FieldVector{6,Float64}
    y::Float64
    z::Float64
    θ::Float64
    ẏ::Float64
    ż::Float64
    θ̇::Float64
end
Quad2DState = Quad2DState2

# ------------------------------------------------

mutable struct Quad2DActuatorCmd2 <: FieldVector{2,Float64}
    left_motor_thrust::Float64
    right_motor_thrust::Float64
end

Quad2DActuatorCmd = Quad2DActuatorCmd2

# ------------------------------------------------

mutable struct Quad2DControlCmd4 <: FieldVector{2,Float64}
    body_thrust::Float64
    body_torque::Float64
end

Quad2DControlCmd = Quad2DControlCmd4

# ------------------------------------------------
struct Quad2D2 <: FrRobot
    nx::Int
    nu::Int

    state::Quad2DState
    params::NamedTuple
end

Quad2D = Quad2D2

# ------------------------------------------------
struct Quad2DController1 <: FrDigitalController
    allocation_matrix::SMatrix
end

Quad2DController = Quad2DController1

# ------------------------------------------------
