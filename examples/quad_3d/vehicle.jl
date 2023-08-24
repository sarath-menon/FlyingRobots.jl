
function load_vehicle_params(path::String)
    vehicle_yaml = YAML.load_file(folder_path * path; dicttype=Dict{Symbol,Any})

    # # set tasks per ticks for each computer task
    for task in vehicle_yaml[:computer][:tasks]
        clock_speed = vehicle_yaml[:computer][:clock_speed]
        task_rate_hz = task[:rate]
        @show task[:rate_per_tick] = Int(clock_speed / task_rate_hz)
    end

    vehicle_params = recursive_dict_to_namedtuple(vehicle_yaml)

    return vehicle_params
end

function load_sim_params(path::String, vehicle_params)
    sim_yaml = YAML.load_file(folder_path * "/parameters/sim.yml"; dicttype=Dict{Symbol,Any})

    # set integrator callback rate
    sim_yaml[:callback_dt] = 1 / vehicle_params.computer.clock_speed

    sim_params = recursive_dict_to_namedtuple(sim_yaml)

    return sim_params
end


function load_controller_params(path::String)
    ctrl_yaml = YAML.load_file(folder_path * path; dicttype=Dict{Symbol,Any})
    return ctrl_yaml
end



# "/parameters/vehicle.yml"