

macro get_sim_params(sim_params_path)
    return :(load_vehicle_params(folder_path * $params_paths["vehicle"]))
end

let

    selv = @pre_sim_setup params_paths

    @show selv

end
