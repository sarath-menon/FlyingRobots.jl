export_run_sim

using DifferentialEquations

function run_sim!(model, logger, tspan, initial_conditions, params, control_cb)

    reset!(logger)

    prob = ODEProblem(model, initial_conditions, tspan, params, callback=control_cb)

    # solve ODE
    #@time sol = solve(prob, Tsit5(), abstol=1e-8, reltol=1e-8, save_everystep=false, save_on=false)
    sol = solve(prob, Tsit5(), abstol=1e-8, reltol=1e-8, save_everystep=false, save_end=false)

    return nothing
end