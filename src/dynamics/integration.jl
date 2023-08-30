function quaternion_integrator(q0, ω, dt)
    q_exp = exp(QuatRotation(0, ω[1] * dt / 2, ω[2] * dt / 2, ω[3] * dt / 2, false).q)
    q_exp_q = QuatRotation(q_exp.s, q_exp.v1, q_exp.v2, q_exp.v3, false)

    q1 = q_exp_q * q0

    return q1
end


