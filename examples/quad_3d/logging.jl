
function log_sim(sol)

    df = DataFrame(sol)

    # dataframe header formatting
    rename!(df, replace.(names(df), r"getindex" => ""))
    rename!(df, replace.(names(df), r"₊" => "."))

    # save as CSV file
    log_path = "logs/"

    # get date-time as string
    date = Dates.format(Dates.now(), "e,dd-u-yyyy HH:MM:SS")

    log_folder_path = log_path * date

    # create folder with date-time as name
    Base.Filesystem.mkdir(log_folder_path)
    Base.Filesystem.mkdir(log_folder_path * "/parameters")

    CSV.write(log_folder_path * "/sim_output.csv", df)

    # copy the parameer files 
    Base.Filesystem.cp(folder_path * vehicle_params_path, log_folder_path * vehicle_params_path)
    Base.Filesystem.cp(folder_path * ctrl_yaml_path, log_folder_path * ctrl_yaml_path)
    Base.Filesystem.cp(folder_path * sim_params_yaml, log_folder_path * sim_params_yaml)

end