export quad_2d_dynamics_diffeq

function quad_2d_dynamics(X, U, params::NamedTuple)

    # extract the parameters
    m, l, I_xx, safety_box, K = params.quad

    g_vec = SA_F64[0; g] # use static array

    y = X[1]
    z = X[2]
    θ = X[3]
    ẏ = X[4]
    ż = X[5]
    θ̇ = X[6]

    # get the control input
    f_1 = U[1]
    f_2 = U[2]

    # compute thrust, torque
    f_thrust = f_1 + f_2
    a_thrust = (f_thrust / m) # mass normalized thrust 

    τ = (f_1 - f_2) * l

    # translation E.O.M
    f = SA_F64[0; a_thrust]
    (ÿ, z̈) = R_2D(θ) * f + g_vec

    # rotational E.O.M
    θ̈ = τ / I_xx

    return SA_F64[ẏ, ż, θ̇, ÿ, z̈, θ̈]
end

#Define the problem
function quad_2d_dynamics_diffeq(d_state::Vector{Float64}, state::Vector{Float64}, params::NamedTuple, t)

    # Extract the parameters
    m, l, I_xx, safety_box, K = params.quad
    nx, nu, ny, Ts = params.frmodel

    # extract the state
    X = @view state[1:nx]

    # extract the control input
    U = @view state[nx+1:end]

    (ẏ, ż, θ̇, ÿ, z̈, θ̈) = quad_2d_dynamics(X, U, params)

    d_state[1], d_state[2], d_state[3] = ẏ, ż, θ̇
    d_state[4], d_state[5], d_state[6] = ÿ, z̈, θ̈

    return nothing
end
