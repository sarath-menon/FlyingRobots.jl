using ModelingToolkit
using Symbolics


function convert_to_numeric(A::Array{Num})
    A_num = [i.val for i in A]
    A_num = convert(Array{Float64}, A_num)

    return A_num
end

function linearize_system(Ts::Float64, x₀::Pose2D, quad_obj::Quad2d, u₀_vec::Array{Float64})
    # ================= Copy pasted , don't modify ! ========================

    # define parameters and variables
    @parameters l, I_xx, m

    # state variables
    @variables t, y(t), z(t), θ(t)

    # control input
    @variables f_1(t) = 0 [input = true] f_2(t) = 0 [input = true]

    @constants g = -9.81

    # define operators
    D = Differential(t)

    f_thrust = f_1 + f_2
    a_thrust = (f_thrust / m) # mass normalized thrust 

    τ = (f_1 - f_2) * l

    # dynamics equations

    # gravity 
    g_vec = [0; g]

    # create custom rotation matrix
    R_2D(θ::Real) = [cos(θ) -sin(θ); sin(θ) cos(θ)]

    # translation E.O.M
    f = [0; a_thrust]
    (ÿ, z̈) = R_2D(θ) * f + g_vec

    # rotational E.O.M
    θ̈ = τ / I_xx

    eqs = [
        D(D(y)) ~ ÿ,
        D(D(z)) ~ z̈,
        D(D(θ)) ~ θ̈]

    # create ODESystem and simplify
    @named sys_unsimplified = ODESystem(eqs)

    # ==================================================================== 

    nonlinear_eq_vec = [D(y),
        D(z),
        D(θ),
        equations(sys_unsimplified)[1].rhs,
        equations(sys_unsimplified)[2].rhs,
        equations(sys_unsimplified)[3].rhs]

    state_vec = [y, z, θ, D(y), D(z), D(θ)]
    control_vec = [f_1, f_2]

    # Calculate A,B matrices
    A = Symbolics.jacobian(nonlinear_eq_vec, state_vec)
    B = Symbolics.jacobian(nonlinear_eq_vec, control_vec)

    AB_symbolic = [A, B]


    # Linearize around operating point
    x₀ = Dict(
        y => x₀.y,
        z => x₀.z,
        θ => x₀.θ,
        D(y) => x₀.ẏ,
        D(z) => x₀.ż,
        D(θ) => x₀.θ̇,)

    u₀ = Dict(
        f_1 => u₀_vec[1],
        f_2 => u₀_vec[2],
    )

    A = substitute(A, u₀)
    A = substitute(A, x₀)
    B = substitute(B, u₀)
    B = substitute(B, x₀)

    # parameter values
    params = Dict(
        l => quad_obj.L,
        I_xx => quad_obj.I_xx,
        m => quad_obj.m,
    )

    # Assume y,z positon and angular velocity is measured
    C = [1 1 0 0 0 1]
    D = 0

    matrices_symbolic = [A, B, C, D]

    # Substitute parameters
    A = substitute(A, params)
    B = substitute(B, params)

    A = convert_to_numeric(A)
    B = convert_to_numeric(B)

    # create continuous time system 
    sys_c = ss(A, B, C, 0)

    # create discrete time system 
    sys_d = c2d(sys_c, Ts)

    return sys_c, sys_d, matrices_symbolic

end