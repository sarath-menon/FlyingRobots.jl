function quaternion_integrator(q0, ω, dt)
    q_exp = exp(QuatRotation(0, ω[1] * dt / 2, ω[2] * dt / 2, ω[3] * dt / 2, false).q)
    q_exp_q = QuatRotation(q_exp.s, q_exp.v1, q_exp.v2, q_exp.v3, false)

    q1 = q_exp_q * q0

    return q1
end

function RigidBody(; name, m, I_xx, I_yy, I_zz)

    # translation
    @variables t (r(t))[1:3] = 0 (ṙ(t))[1:3] = 0

    # rotation
    @variables (q(t))[1:4] = 0 (ω(t))[1:3] = 0

    # external force, torque
    @variables (f(t))[1:3] = 0 (τ(t))[1:3]

    @parameters m = m I_xx = I_xx I_yy = I_yy I_zz = I_yy

    D = Differential(t)

    translational_kinematics = D.(r) .~ ṙ

    translational_dynamics = D.(ṙ) .~ f / m

    rotation_kinematics = D.(q) .~ 0  # implemented in callback

    rotation_dynamics = [
        D.(ω[1]) .~ (τ[1] - (I_yy - I_zz) * ω[2] * ω[3]) / I_xx,
        D.(ω[2]) .~ (τ[2] - (I_xx - I_zz) * ω[1] * ω[3]) / I_yy,
        D.(ω[3]) .~ (τ[3] - (I_yy - I_xx) * ω[1] * ω[2]) / I_zz,
    ]

    # eqns = vcat(translational_kinematics,translational_dynamics, rotation_kinematics, rotation_dynamics, external_force ,external_torque)
    eqns = vcat(translational_kinematics, translational_dynamics, rotation_kinematics, rotation_dynamics)

    ODESystem(eqns, t; name)

    # sys = structural_simplify(model);

end

motor_thrust_to_body_thrust(l, k_τ) = [1 1 1 1
    0 l 0 -l
    -l 0 l 0
    k_τ -k_τ k_τ -k_τ]

body_thrust_to_motor_thrust(l, k_τ) = inv(motor_thrust_to_body_thrust(l, k_τ))