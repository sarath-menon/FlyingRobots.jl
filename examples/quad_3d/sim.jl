
function load_sim_params(path::String, vehicle_params)
    sim_yaml = YAML.load_file(folder_path * "/parameters/sim.yml"; dicttype=Dict{Symbol,Any})

    # set integrator callback rate equal to clock speed
    sim_yaml[:callback_dt] = 1 / vehicle_params.computer.clock_speed

    sim_params = recursive_dict_to_namedtuple(sim_yaml)

    return sim_params
end