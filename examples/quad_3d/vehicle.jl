
function load_vehicle_params(path::String)
    vehicle_yaml = YAML.load_file(folder_path * path; dicttype=Dict{Symbol,Any})

    # # set tasks per ticks for each computer task
    # task_rates = vehicle_params.computer.task_rates
    # tasks_per_ticks = get_ticks_per_task(task_rates)

    vehicle_params = recursive_dict_to_namedtuple(vehicle_yaml)

    return vehicle_params
end

function load_sim_params(path::String)
    sim_yaml = YAML.load_file(folder_path * "/parameters/sim.yml"; dicttype=Dict{Symbol,Any})

    # set integrator callback rate
    sim_yaml[:callback_dt] = 1 / vehicle_yaml[:computer][:clock_speed]

    sim_params = recursive_dict_to_namedtuple(sim_yaml)

    return sim_params
end


function load_controller_params(path::String)
    ctrl_yaml = YAML.load_file(folder_path * path; dicttype=Dict{Symbol,Any})
    return ctrl_yaml
end





# "/parameters/vehicle.yml"