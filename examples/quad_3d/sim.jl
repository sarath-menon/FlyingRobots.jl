
function load_sim_params(path::String, vehicle_params)
    sim_yaml = YAML.load_file(folder_path * "/parameters/sim.yml"; dicttype=Dict{Symbol,Any})

    # set integrator callback rate equal to clock speed
    sim_yaml[:callback_dt] = 1 / vehicle_params.computer.clock_speed

    sim_params = recursive_dict_to_namedtuple(sim_yaml)

    return sim_params
end

function run_sim(sys, subsystems, sim_params, vehicle_params; save=false)

    cb = PeriodicCallback(integrator_callback, sim_params.callback_dt, initial_affect=true, save_positions=(true, false))

    tspan = (0.0, sim_params.duration)

    X₀ = get_initial_conditions(subsystems[:plant], vehicle_params)
    parameters = get_parameters(subsystems[:plant], vehicle_params)

    reset_pid_controllers()

    prob = ODEProblem(sys, X₀, tspan, parameters, callback=cb)
    sol = solve(prob, Tsit5(), abstol=1e-8, reltol=1e-8, save_everystep=false)

    if save == true
        df = DataFrame(sol)

        # dataframe header formatting
        rename!(df, replace.(names(df), r"getindex" => ""))
        rename!(df, replace.(names(df), r"₊" => "."))

        # save as CSV file
        CSV.write("logs/log.csv", df)

    end

    return sol
end