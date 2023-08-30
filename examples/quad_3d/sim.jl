
function load_sim_params(path::String, vehicle_params)
    sim_yaml = YAML.load_file(path; dicttype=Dict{Symbol,Any})

    # set integrator callback rate equal to clock speed
    sim_yaml[:callback_dt] = 1 / vehicle_params.computer.clock_speed

    sim_params = recursive_dict_to_namedtuple(sim_yaml)

    return sim_params
end

function run_sim(sys, subsystems; save=false)

    prob = sim_setup(sys, subsystems)

    @time sol = solve(prob, Tsit5(), abstol=1e-8, reltol=1e-8, save_everystep=false)

    df = sim_logging(integrator.sol)

    return df
end



function run_sim_stepping(sys, subsystems; save=false)

    prob = sim_setup(sys, subsystems)

    integrator = init(prob, Tsit5(), abstol=1e-8, reltol=1e-8, save_everystep=false)

    while floor(integrator.t) <= prob.tspan[2]
        step!(integrator)
    end

    df = sim_logging(integrator.sol)

    return df
end

# pre-sim setup
function sim_setup(sys, subsystems)
    # load params
    vehicle_params = load_vehicle_params(folder_path * vehicle_params_path)
    sim_params = load_sim_params(folder_path * sim_params_path, vehicle_params)

    condition(u, t, integrator) = true

    control_cb = PeriodicCallback(computer_cycle, sim_params.callback_dt, initial_affect=true, save_positions=(false, true))
    integrator_cb = DiscreteCallback(condition, integrator_callback, save_positions=(false, false))

    cb_set = CallbackSet(integrator_cb, control_cb)

    tspan = (0.0, sim_params.duration)

    X₀ = get_initial_conditions(subsystems.plant, vehicle_params)
    parameters = get_parameters(subsystems.plant, vehicle_params)

    prob = ODEProblem(sys, X₀, tspan, parameters, callback=cb_set)

    return prob
end

function sim_logging(sol)
    df = DataFrame(sol)
    # dataframe header formatting
    rename!(df, replace.(names(df), r"getindex" => ""))
    rename!(df, replace.(names(df), r"₊" => "."))

    if save == true
        log_sim(df)
    end

    return df
end