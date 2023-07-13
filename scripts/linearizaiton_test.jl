module Waste

using ForwardDiff

function dynamics(X, U)

    # extract the parameters
    m, l, I_xx = params

    y = X[1]
    z = X[2]
    θ = X[3]
    ẏ = X[4]
    ż = X[5]
    θ̇ = X[6]

    # get the control input
    body_thrust = U[1]
    body_torque = U[2]

    # f = @SVector [0.0, 0.0]
    f = [0.0, body_thrust]

    # gravity vector 
    g_vec = [0; g] # use static array

    # rotational E.O.M
    ÿ, z̈ = R_2D(θ) * f + g_vec

    # rotational E.O.M
    θ̈ = body_torque / I_xx

    return [ẏ, ż, θ̇, ÿ, z̈, θ̈]
end

translational_dynamics(var) = translational_dynamics(var[1], [var[2], var[2]])

# vehicle parameters
params = (; m=1.0, l=0.2, I_xx=0.003)

# operating pint to linearize around
x0 = [0, 0, 0, 0, 0, 0]
u0 = [0, 0]

A = ForwardDiff.jacobian(x -> dynamics(x, u0), x0)
B = ForwardDiff.jacobian(u -> dynamics(x0, u), u0)


end