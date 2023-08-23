
function Quadcopter(; name, l, k_τ, m, I_xx, I_yy, I_zz)

    # translation
    @variables t (r(t))[1:3] = 0 (ṙ(t))[1:3] = 0

    # rotation
    @variables (q(t))[1:4] = 0 (ω(t))[1:3] = 0

    # motor thrusts
    @variables t (f(t))[1:4] = 0

    params = @parameters l = l k_τ = k_τ g = 9.81

    @named rb = RigidBody(; name=:rb, m=m, I_xx=I_xx, I_yy=I_yy, I_zz=I_yy)

    # forces
    f_x = 0
    f_y = 0
    f_z = f[1] + f[2] + f[3] + f[4]

    # moments
    τ_x = l * (f[2] - f[4])
    τ_y = l * (f[3] - f[1])
    τ_z = k_τ * (f[1] - f[2] + f[3] - f[2])

    # transform from body to intertial frame
    R_IB = QuatRotation(q[1], q[2], q[3], q[4], false)

    f_net = R_IB * [f_x; f_y; f_z] - [0; 0; g] - 0.1 .* [ṙ[1]; ṙ[2]; ṙ[3]]

    thrust_eqn = rb.f .~ f_net
    torque_eqn = rb.τ .~ [τ_x; τ_y; τ_z]


    # set the quadcopter pose to equal the rigidbody pose
    eqn3 = r .~ rb.r
    eqn4 = ṙ .~ rb.ṙ
    eqn5 = q .~ rb.q
    eqn6 = ω .~ rb.ω

    eqns = vcat(thrust_eqn, torque_eqn, eqn3, eqn4, eqn5, eqn6)

    # connect the subsystems
    ODESystem(eqns, t, systems=[rb]; name)

end