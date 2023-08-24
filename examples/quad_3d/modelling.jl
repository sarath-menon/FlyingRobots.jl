

function build_system_model()

    ## initialize subsystems
    @named plant = Quadcopter(; name=:quad1)
    @named controller = Controller_Zero_Order_Hold()

    # motor thrusts
    eqn1 = controller.U .~ plant.f_cmd

    # connect the subsystems
    eqns = vcat(eqn1)
    @named sys_unsimplified = ODESystem(eqns,
        systems=[plant, controller])

    sys = structural_simplify(sys_unsimplified)

    subsystems = Dict()

    subsystems[:plant] = plant
    subsystems[:controller] = controller

    return sys, subsystems
end