
export dynamics!, dynamics_diffeq!, update_state!
export actuator_cmd_to_ctrl_cmd
export translational_dynamics


function dynamics!(quad_2d::Quad2D, control_cmd::Quad2DControlCmd)

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

    a_thrust = (control_cmd.body_thrust / m) # mass normalized body thrust 
    τ = control_cmd.body_torque

    # gravity vector 
    g_vec = SA_F64[0; g] # use static array

    # translation E.O.M
    f = SA_F64[0; a_thrust]
    (ÿ, z̈) = R_2D(θ) * f + g_vec

    # rotational E.O.M
    θ̈ = τ / I_xx

    return SA_F64[ẏ, ż, θ̇, ÿ, z̈, θ̈]
end

function dynamics!(quad_2d::Quad2D, X, U)

    update_state!(quad_2d, X)
    control_cmd = fr_comm(U, Quad2DState)

    dynamics!(quad_2d, control_cmd)
end


function update_state!(quad_2d::Quad2D, state::Vector{Float64})
    quad_2d.state.y = state[1]
    quad_2d.state.z = state[2]
    quad_2d.state.θ = state[3]
    quad_2d.state.ẏ = state[4]
    quad_2d.state.ż = state[5]
    quad_2d.state.θ̇ = state[6]
end

function update_state!(quad_2d::Quad2D, state::Quad2DState)
    quad_2d.state.y = state.y
    quad_2d.state.z = state.z
    quad_2d.state.θ = state.θ
    quad_2d.state.ẏ = state.ẏ
    quad_2d.state.ż = state.ż
    quad_2d.state.θ̇ = state.θ̇
end


function dynamics_diffeq!(d_diffeq_state, diffeq_state, params::NamedTuple, t)

    # Extract the parameters
    quad_2d = params.FrRobot

    # extract the state
    state_vec = @view diffeq_state[1:quad_2d.nx]

    # extract the control input
    control_vec = @view diffeq_state[quad_2d.nx+1:end]

    update_state!(quad_2d, state_vec)

    control_cmd = Quad2DControlCmd(control_vec[1], control_vec[2])

    (ẏ, ż, θ̇, ÿ, z̈, θ̈) = dynamics!(quad_2d, control_cmd)

    d_diffeq_state[1], d_diffeq_state[2], d_diffeq_state[3] = ẏ, ż, θ̇
    d_diffeq_state[4], d_diffeq_state[5], d_diffeq_state[6] = ÿ, z̈, θ̈

    return nothing

end

function actuator_cmd_to_ctrl_cmd(quad_2d::Quad2D, actuator_cmd::Quad2DActuatorCmd)

    # extract the parameters
    m, L, I_xx = quad_2d.params

    # f_left = actuator_cmd.left_motor_thrust
    # f_right = actuator_cmd.right_motor_thrust

    actuator_cmd_vec = @SVector[actuator_cmd.left_motor_thrust, actuator_cmd.right_motor_thrust]

    # # compute body  thrust
    # body_thrust = f_left + f_right
    # mass_normalized_body_thrust = (body_thrust / m) # mass normalized body  thrust 

    # # compute body thrust 
    # body_torque = (f_left - f_right) * L

    control_cmd_vec = actuator_cmd_to_ctrl_cmd_matrix(L) * actuator_cmd_vec
    control_cmd = fr_create(Quad2DControlCmd; body_thrust=control_cmd_vec[1], body_torque=control_cmd_vec[2])

    return control_cmd
end





function apply_control_cmd!(quad_2d::Quad2D, control_cmd)

end

