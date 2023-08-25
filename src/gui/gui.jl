
module Gui

using GLMakie
using GeometryBasics
using Rotations
using FileIO
using YAML
using StaticArrays
using DataFrames

include("../common/types.jl")
include("../common/utilities/utilities.jl")

include("gui_helper.jl")
include("gui_tasks.jl")
include("gui_utilities.jl")

export show_visualizer, PlotData, plot_reset, plot_3d_trajectory
export set_sim_instance

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

end