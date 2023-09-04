
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

function get_empty_df(sys, subsystems)
    prob = sim_setup(sys, subsystems)
    integrator = init(prob, Tsit5(), abstol=1e-8, reltol=1e-8, save_everystep=false)
    df_empty = sim_logging(integrator.sol)

    return df_empty
end

function run_sim_stepping(sys, subsystems; save=false)

    prob = sim_setup(sys, subsystems)

    integrator = init(prob, Tsit5(), abstol=1e-8, reltol=1e-8, save_everystep=false)

    counter::Int64 = 0

    # perform the integration
    for i in integrator
    end

    df = sim_logging(integrator.sol)

    return df
end

function run_sim_stepping(sys, subsystems, c1, flag, sim_acc_state; save=false, physical_time=false)

    prob = sim_setup(sys, subsystems)

    integrator = init(prob, Tsit5(), abstol=1e-8, reltol=1e-8, save_everystep=false)

    count_prev::Int64 = 0
    buffer_size = 10

    t_start = time_ns()

    # perform the integration
    for (u, t) in tuples(integrator)

        if sim_acc_state[] == RealtimeSim()

            counter = length(integrator.sol.t)

            if counter % buffer_size == 0 && counter > count_prev
                # Core.println( integrator.sol.t[end])
                sol_subset = integrator.sol[end-(buffer_size-1):end]
                put!(c1, sol_subset)

                count_prev = counter

                if flag[] == false
                    Core.println("Terminting integration")
                    break
                end
            end

            t_now = time_ns()
            real_time_elapsed = (t_now - t_start) * 1e-9
            sim_time_elapsed = t

            t_wait = sim_time_elapsed - real_time_elapsed

            sleep(maximum([0.0, t_wait]))

        end
    end

    flag[] = false

    df = sim_logging(integrator.sol)

    return df
end

# pre-sim setup
function setup_callbacks(sim_params)
    condition(u, t, integrator) = true

    control_cb = PeriodicCallback(computer_cycle, sim_params.callback_dt, initial_affect=true, save_positions=(false, true))
    integrator_cb = DiscreteCallback(condition, integrator_callback, save_positions=(false, false))

    cb_set = CallbackSet(integrator_cb, control_cb)

    return cb_set
end

function sim_setup(sys, subsystems)
    # load params
    vehicle_params = load_vehicle_params(folder_path * vehicle_params_path)
    sim_params = load_sim_params(folder_path * sim_params_path, vehicle_params)

    cb_set = setup_callbacks(sim_params)

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