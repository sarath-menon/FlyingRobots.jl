
## Systems test
module SystemsTest

using FlyingRobots
using ControlSystems
using StaticArrays
using Test

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

# Create sample Robot 
struct Quad2D3 <: FrRobot
    nx::Int
    nu::Int

    state::Quad2DState
    control_cmd::Quad2DControlCmd
    params::NamedTuple
end

Quad2D = Quad2D3

quad_2d_params = (; m=1.0, L=0.1, I_xx=0.003)

intial_state = Quad2DState(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
intial_control_cmd = Quad2DControlCmd(0.0, 0.0)

quad_2d = Quad2D(6, 2, intial_state, intial_control_cmd, quad_2d_params)


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

function update_state!(quad_2d::Quad2D, state)


    quad_2d.state.y = state[1]
    quad_2d.state.z = state[2]
    quad_2d.state.θ = state[3]
    quad_2d.state.ẏ = state[4]
    quad_2d.state.ż = state[5]
    quad_2d.state.θ̇ = state[6]
end


function dynamics(d_state::Vector{Float64}, state::Vector{Float64}, params::NamedTuple, t)

    # Extract the parameters
    quad_2d = params.FrRobot

    # extract the state
    X = @view state[1:quad_2d.nx]

    # extract the control input
    U = @view state[quad_2d.nx+1:end]

    update_state!(quad_2d, X)

    # dynamics(quad_2d)

end


dynamics(quad_2d)

# testing
let
    quad_2d_params = (; m=1.0, L=0.1, I_xx=0.003)
    intial_state = Quad2DState(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    intial_control_cmd = Quad2DControlCmd(0.0, 0.0)
    quad_2d = Quad2D(6, 2, intial_state, intial_control_cmd, quad_2d_params)

    new_state = rand(quad_2d.nx)

    # test update state_func
    update_state!(quad_2d, new_state)
    @test check_if_struct_equals_vector(quad_2d.state, new_state)
end


# state = rand(quad_2d.nx)
# t::Float64 = rand()
# params = (; FrRobot=quad_2d)
# dynamics(d_state, state, params, t)






end