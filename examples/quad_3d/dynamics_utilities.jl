function quaternion_integrator(q0, ω, dt)
    q_exp = exp(QuatRotation(0, ω[1] * dt / 2, ω[2] * dt / 2, ω[3] * dt / 2, false).q)
    q_exp_q = QuatRotation(q_exp.s, q_exp.v1, q_exp.v2, q_exp.v3, false)

    q1 = q_exp_q * q0

    return q1
end



motor_thrust_to_body_thrust(; l, k_τ) = [1 1 1 1
    0 l 0 -l
    -l 0 l 0
    k_τ -k_τ k_τ -k_τ]

body_thrust_to_motor_thrust(l, k_τ) = inv(motor_thrust_to_body_thrust(; l=l, k_τ=k_τ))