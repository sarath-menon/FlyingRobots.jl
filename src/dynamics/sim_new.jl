export quad_2d_dynamics_diffeq_new

function quad_2d_dynamics_new(state::Quad2DState, actuator_cmd::Quad2DActuatorCmd, params::NamedTuple)

    # extract the parameters
    m, l, I_xx, safety_box, K = params.quad

    g_vec = @SVector [0; g] # use static array

    #extract the state
    X = state
    y = X.y
    z = X.z
    θ = X.θ
    ẏ = X.ẏ
    ż = X.ż
    θ̇ = X.θ̇

    # get the control input
    f_1 = actuator_cmd.left_motor_thrust
    f_2 = actuator_cmd.right_motor_thrust

    # compute thrust, torque
    f_thrust = f_1 + f_2
    a_thrust = (f_thrust / m) # mass normalized thrust 

    τ = (f_1 - f_2) * l

    # translation E.O.M
    f = @SVector [0; a_thrust]
    (ÿ, z̈) = R_2D(θ) * f + g_vec

    # rotational E.O.M
    θ̈ = τ / I_xx

    return @SVector [ẏ, ż, θ̇, ÿ, z̈, θ̈]
end

#Define the problem
function quad_2d_dynamics_diffeq_new(d_state::Vector{Float64}, state::Vector{Float64}, params::NamedTuple, t)

    # Extract the parameters
    m, l, I_xx, safety_box, K = params.quad
    nx, nu, ny, Ts = params.frmodel

    # extract the state
    X = @view state[1:nx]

    # extract the control input
    U = @view state[nx+1:end]

    state_ = convert(Quad2DState, X)
    ctrl_cmd = convert(Quad2DActuatorCmd, U)

    # @show state

    (ẏ, ż, θ̇, ÿ, z̈, θ̈) = quad_2d_dynamics_new(state_, ctrl_cmd, params)

    d_state[1], d_state[2], d_state[3] = ẏ, ż, θ̇
    d_state[4], d_state[5], d_state[6] = ÿ, z̈, θ̈

    return nothing
end
