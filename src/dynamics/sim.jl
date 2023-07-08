function quad_2d_dynamics(X::Vector{Float64}, U::Vector{Float64}, params::NamedTuple)

    # extract the parameters
    m, g, l, I_xx = params

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

    return (ẏ, ż, θ̇, ÿ, z̈, θ̈)
end

#Define the problem
function quad_2d(d_state, state, params, t)

    # extract the state
    X = state[1:sim_params.nx]

    # extract the control input
    U = state[sim_params.nx+1:end]

    (ẏ, ż, θ̇, ÿ, z̈, θ̈) = quad_2d_dynamics(X, U, params)

    d_state[1], d_state[2], d_state[3] = ẏ, ż, θ̇
    d_state[4], d_state[5], d_state[6] = ÿ, z̈, θ̈
end

# run at every timestep
condition(u, t, integrator) = true

function affect!(integrator)

    # Extract the state 
    X = integrator.u[1:sim_params.nx]

    y::Float64 = X[1]
    z::Float64 = X[2]
    θ::Float64 = X[3]
    ẏ::Float64 = X[4]
    ż::Float64 = X[5]
    θ̇::Float64 = X[6]

    # # Limit operating space 
    # if z > params.safety_box.z_max
    #     z = clamp(z, params.safety_box.z_min, params.safety_box.z_max)
    #     ż = 0
    #     println("Z operational space constraint violated !")
    #     terminate!(integrator)

    # elseif z < params.safety_box.z_min
    #     z = clamp(z, params.safety_box.z_min, params.safety_box.z_max)
    #     ż = 0
    #     println("Landed !")
    #     terminate!(integrator)
    # end

    # if y < params.safety_box.y_min || y > params.safety_box.y_max
    #     y = clamp(y, params.safety_box.y_min, params.safety_box.y_max)
    #     ẏ = 0
    #     println("Y operational space constraint violated !")
    #     terminate!(integrator)
    # end

end

cb = DiscreteCallback(condition, affect!)