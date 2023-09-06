

function Quadcopter(; name)

    # translation
    @variables t (r(t))[1:3] = 0 (ṙ(t))[1:3] = 0

    # rotation
    @variables (q(t))[1:4] = 0 (ω(t))[1:3] = 0

    # motor thrusts
    @variables t (f_cmd(t))[1:4] = 0 [input = true] (f(t))[1:4] = 0

    # params = @parameters l = l k_τ = k_τ g = 9.81
    params = @parameters l k_τ g = 9.81

    @named rb = RigidBody(; name=:rb)

    # motors
    @named motor_1 = BldcMotorPropellerPair(; name=:motor_1)
    @named motor_2 = BldcMotorPropellerPair(; name=:motor_2)
    @named motor_3 = BldcMotorPropellerPair(; name=:motor_3)
    @named motor_4 = BldcMotorPropellerPair(; name=:motor_4)


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

    # connect thrust commands to motor thrust inputs
    motor_eqns = [
        motor_1.thrust_cmd ~ f_cmd[1],
        motor_2.thrust_cmd ~ f_cmd[2],
        motor_3.thrust_cmd ~ f_cmd[3],
        motor_4.thrust_cmd ~ f_cmd[4],
        motor_1.thrust_output ~ f[1],
        motor_2.thrust_output ~ f[2],
        motor_3.thrust_output ~ f[3],
        motor_4.thrust_output ~ f[4],
    ]

    # set the quadcopter pose to equal the rigidbody pose
    eqn3 = r .~ rb.r
    eqn4 = ṙ .~ rb.ṙ
    eqn5 = q .~ rb.q
    eqn6 = ω .~ rb.ω

    eqns = vcat(motor_eqns, thrust_eqn, torque_eqn, eqn3, eqn4, eqn5, eqn6)

    # connect the subsystems
    ODESystem(eqns, t, systems=[rb, motor_1, motor_2, motor_3, motor_4]; name)

end


# force control input to be constant over sampling time, actual control law applied inside callback
function Controller_Zero_Order_Hold(; name)
    sts = @variables t U(t)[1:4] = 0 (R(t))[1:3] = 0

    # define operators
    D = Differential(t)

    eqn1 = D.(U) .~ 0
    eqn2 = D.(R) .~ 0

    eqns = vcat(eqn1, eqn2)

    ODESystem(eqns, t; name)
end


function QuadcopterSystem()

    ## initialize subsystems
    @named quadcopter = Quadcopter(; name=:quad1)
    @named controller = Controller_Zero_Order_Hold()

    # motor thrusts
    eqn1 = controller.U .~ quadcopter.f_cmd

    # connect the subsystems
    eqns = vcat(eqn1)
    @named sys_unsimplified = ODESystem(eqns,
        systems=[quadcopter, controller])

    sys = structural_simplify(sys_unsimplified)

    subsystems = (; plant=quadcopter, controller=controller)

    return sys, subsystems
end