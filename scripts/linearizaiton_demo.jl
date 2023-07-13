module Waste

using ForwardDiff
using FlyingRobots
using StaticArrays
using ControlSystems

include("./../examples/quad_2d/quad_2d.jl")
using .Quad2D_Demo

# helper functions
vec(X::Quad2DState) = @SVector [X.y, X.z, X.θ, X.ẏ, X.ż, X.θ̇]
vec(X::Quad2DControlCmd) = @SVector [X.body_thrust, X.body_torque]

function dynamics(X, U, params::NamedTuple)

    # extract the parameters
    m, L, I_xx = params

    y = X[1]
    z = X[2]
    θ = X[3]
    ẏ = X[4]
    ż = X[5]
    θ̇ = X[6]

    # get the control input
    body_thrust = U[1]
    body_torque = U[2]

    f = @SVector [0.0, body_thrust]

    # gravity vector 
    g_vec = @SVector [0; g]

    # rotational E.O.M
    ÿ, z̈ = R_2D(θ) * f + g_vec

    # rotational E.O.M
    θ̈ = body_torque / I_xx

    return @SVector [ẏ, ż, θ̇, ÿ, z̈, θ̈]
end

# operating pint to linearize around
state = fr_create(Quad2DState; y=0.0, z=1.0, θ=0.0, ẏ=0.0, ż=0.0, θ̇=0.0)
control_cmd = fr_create(Quad2DControlCmd; body_thrust=-g, body_torque=0.0)

# vehicle parameters
quad_2d_params = (; m=1.0, L=0.1, I_xx=0.003)
quad_2d = fr_create(Quad2D; nx=6, nu=2, state=state, params=quad_2d_params)


function linearize_model(dynamics, robot::FrRobot, state::FrRobotState, control_cmd::FrCtrlCmd)
    x0 = vec(state)
    u0 = vec(control_cmd)

    A = ForwardDiff.jacobian(x -> dynamics(x, u0, params), x0)
    B = ForwardDiff.jacobian(u -> dynamics(x0, u, params), u0)

    nx = robot.nx
    nu = robot.nu

    # Assume that all the states are observale is C is not provided
    C = ones(nu, nx)

    # create continuous time linear system
    sys_c = ss(A, B, C, 0)

    return sys_c
end

# discretize to obtain discrete time system

@time sys_c = linearize_model(dynamics, quad_2d, state, control_cmd)

sys_d = c2d(sys_c, 0.01)

sys_c
end