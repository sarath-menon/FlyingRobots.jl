
module Gui

using GLMakie
using GeometryBasics
using Rotations
using FileIO
using YAML
using StaticArrays

include("../common/types.jl")
include("../common/utilities/utilities.jl")

include("gui_helper.jl")
include("gui_tasks.jl")



export show_visualizer, PlotData, plot_reset, plot_3d_trajectory

set_theme!(
    #     font = "Arial", # inherited by layoutables if not overridden
    fontsize=25, # inherited by layoutables if not overridden
)

# read settings file 
folder_path = pwd() * "/examples/quad_3d"

# plot_yaml = YAML.load_file(folder_path * "/parameters/plot.yml"; dicttype=Dict{Symbol,Any})
vehicle_yaml = YAML.load_file(folder_path * "/parameters/vehicle.yml"; dicttype=Dict{Symbol,Any})

vehicle_params = recursive_dict_to_namedtuple(vehicle_yaml)
# plot_params = recursive_dict_to_namedtuple(plot_yaml)

# load mesh 
crazyflie_stl = load(assetpath(folder_path * "/assets/cf2_assembly.obj"))


# Params --------------------------------------------

# vis_params = plot_params.visualizer
# graph_params = plot_params.graph

configs_vec = get_configs_vec(plot_params.graph.axis.configs)

# to store plot elements
elements = Dict()

# Observables -------------------------------------------------
sim_time = Observable{Float64}(0.0)
sim_state = Observable{Bool}(false)

# # event handling
# on(config_menu.selection) do config
#     plot_trajectory(configs_vec, config)
# end

# on(timeline_btn.clicks) do clicks

#     # if sim is not already running, start sim
#     if sim_state_obs[] == false

#         # set sim state flag to true
#         sim_state_obs[] = true

#         # change button text to show "Stop"
#         timeline_btn.label = "Stop"

#         # start sim task asynchronously
#         sim_task = @async plot_3d_trajectory(duration=3.0, sim_time_obs=sim_time_obs, sim_state_obs=sim_state_obs)
#         #plot_3d_trajectory(duration=3.0, sim_time_obs=sim_time_obs, sim_state_obs=sim_state_obs)

#         @async begin
#             # wait for 3d trajectory plotting to finish
#             wait(sim_task)

#             # set sim state flag to false 
#             sim_state_obs[] = false

#             # change button text to show "Play"
#             timeline_btn.label = "Play"
#         end

#     else
#         # if sim is currently running, set sim state flag to false
#         sim_state_obs[] = false
#     end

# end


# # lift(timeline_slider.value) do val
# #     # sim_time_obs[] = val
# # end

# on(sim_time_obs) do time
#     elements[:time_title].text = "Time: " * string(time) * " s"
# end

# time_marker = lift(sim_time_obs) do time_val
#     time_val
# end



# initial setup
# plot_trajectory(config_dict, config_menu.selection[])
# plot_trajectory(configs_vec, config_menu.selection[])

# traj_count_n = 1000
# lines!(vis_ax, zeros(traj_count_n), df.y_req[1:traj_count_n], df.z_req[1:traj_count_n], linestyle=:dash)

# trim!(f.layout)

# display(f)
end