
## Systems test
module SystemsTest

using FlyingRobots
using ControlSystems
using StaticArrays


abstract type FrSys end

abstract type FrContinuousSys <: FrSys end
abstract type FrDiscreteSys <: FrSys end
abstract type FrHybridSys <: FrSys end

abstract type FrRobot <: FrHybridSys end

abstract type FrDigitalCtrl <: FrDiscreteSys end

abstract type FrRobotDynamics <: FrContinuousSys end

abstract type FrState end
abstract type FrRobotState <: FrState end

abstract type FrCtrlCmd end
abstract type FrRobotCtrlCmd <: FrCtrlCmd end

mutable struct Quad2DState1 <: FrRobotState
    y::Float64
    z::Float64
    θ::Float64
    ẏ::Float64
    ż::Float64
    θ̇::Float64
end

mutable struct Quad2DControlCmd1 <: FrRobotCtrlCmd
    left_motor_thrust::Float64
    right_motor_thrust::Float64
end

Quad2DControlCmd = Quad2DControlCmd1

Quad2DState = Quad2DState1

# parameters
# abstract type FrParams end
# abstract type FrRobotParams<: FrParams end
# abstract type FrControllerParams<: FrParams end

# Create sample Robot 
struct Quad2D2 <: FrRobot
    state::Quad2DState
    control_cmd::Quad2DControlCmd

    params::NamedTuple

end

Quad2D = Quad2D2

quad_2d_params = (; m=1.0, L=0.1, I_xx=0.003)
quad_2d_components = (;)

intial_state = Quad2DState(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
intial_control_cmd = Quad2DControlCmd(0.0, 0.0)

quad_2d = Quad2D(intial_state, intial_control_cmd, quad_2d_params)


function dynamics(quad_2d::Quad2D)

    # extract the parameters
    m, L, I_xx = quad_2d.params

    # extract the state
    X = quad_2d.state
    y = X.y
    z = X.z
    θ = X.θ
    ẏ = X.ẏ
    ż = X.ẏ
    θ̇ = X.ẏ

    # extract the control commands
    control_cmd = quad_2d.control_cmd
    f_left = control_cmd.left_motor_thrust
    f_right = control_cmd.right_motor_thrust

    # compute body  thrust
    f_thrust = f_left + f_right
    a_thrust = (f_thrust / m) # mass normalized body  thrust 

    # compute body thrust 
    τ = (f_left - f_right) * L

    # gravity vector 
    g_vec = SA_F64[0; g] # use static array


    # translation E.O.M
    f = SA_F64[0; a_thrust]
    (ÿ, z̈) = R_2D(θ) * f + g_vec

    # rotational E.O.M
    θ̈ = τ / I_xx

    return SA_F64[ẏ, ż, θ̇, ÿ, z̈, θ̈]

end


dynamics(quad_2d)



end