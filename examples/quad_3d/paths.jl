
import FileWatching: watch_file

# read settings file 
folder_path = pwd() * "/examples/quad_3d"

params_path = folder_path * "/parameters"
vehicle_params_path = "/parameters/vehicle.yml"

sim_params_path = "/parameters/sim.yml"
strategies_yaml_path = "/parameters/strategies.yml"

# vehicle_yaml = YAML.load_file(folder_path * vehicle_params_path; dicttype=Dict{Symbol,Any})

# vehicle_params = load_vehicle_params(folder_path * vehicle_params_path);
# sim_params = load_sim_params(folder_path * sim_params_path, vehicle_params)

function params_auto_updater(params_dict)
    while true
        event = watch_file(params_path)

        if event.changed
            try
                load_params!(params_dict)
                Core.println("Updated parameters")
            catch err
                @warn("Could not read parameter files ")
                @show err
            end
        end
    end
    println("Exited")
    return params_task
end

function load_params!(params_dict)
    params_dict[:vehicle] = load_vehicle_params(folder_path * vehicle_params_path)
    params_dict[:sim] = load_sim_params(folder_path * sim_params_path, params_dict[:vehicle])
    params_dict[:strategies] = load_controller_params(folder_path * strategies_yaml_path, params_dict[:vehicle])
end