
function load_sim_params(path::String, vehicle_params)
    sim_yaml = YAML.load_file(folder_path * "/parameters/sim.yml"; dicttype=Dict{Symbol,Any})

    # set integrator callback rate equal to clock speed
    sim_yaml[:callback_dt] = 1 / vehicle_params.computer.clock_speed

    sim_params = recursive_dict_to_namedtuple(sim_yaml)

    return sim_params
end

function run_sim(sys, sim_params, vehicle_params)

    cb = PeriodicCallback(integrator_callback, sim_params.callback_dt, initial_affect=true)

    tspan = (0.0, sim_params.duration)

    X₀ = get_initial_conditions(vehicle_params)
    parameters = get_parameters(vehicle_params)

    reset_pid_controllers()

    prob = ODEProblem(sys, X₀, tspan, parameters, callback=cb)
    sol = solve(prob, Tsit5(), abstol=1e-8, reltol=1e-8, save_everystep=false)


    return sol
end