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