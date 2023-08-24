

function build_system_model()

    ## initialize subsystems
    @named plant = Quadcopter(; name=:quad1)
    @named controller = Controller_Zero_Order_Hold()

    # motor thrusts
    eqn1 = controller.U .~ plant.f_cmd

    # connect the subsystems
    eqns = vcat(eqn1)
    @named model = ODESystem(eqns,
        systems=[plant, controller])

    sys = structural_simplify(model)

    return sys
end