
module Gui

using GLMakie
using GeometryBasics
using Rotations
using FileIO
using YAML
using StaticArrays
using DataFrames
using DataStructures
using OrdinaryDiffEq

GLMakie.activate!(inline=false)

include("../common/types.jl")
include("../common/utilities/utilities.jl")
include("../simulation/simulation.jl")

include("gui_utilities.jl")
include("gui_interactions.jl")
include("gui_setup.jl")
include("gui_setup_second_window.jl")
include("gui_interactions_second_window.jl")

include("gui_plotting_2d_static.jl")
include("gui_plotting_2d_dynamic.jl")
include("gui_plotting_3d_static.jl")
include("gui_plotting_3d_dynamic.jl")

export show_visualizer, PlotData, plot_reset, plot_3d_trajectory
export set_sim_instance

# read settings file 
folder_path = pwd() * "/examples/quad_3d"

# plot_yaml = YAML.load_file(folder_path * "/parameters/plot.yml"; dicttype=Dict{Symbol,Any})
vehicle_yaml = YAML.load_file(folder_path * "/parameters/vehicle.yml"; dicttype=Dict{Symbol,Any})
vehicle_params = recursive_dict_to_namedtuple(vehicle_yaml)

# second window
config_params_path = "/parameters/menu.yml"

config_yaml = YAML.load_file(folder_path * config_params_path; dicttype=Dict{Symbol,Any})
config_params = recursive_dict_to_namedtuple(config_yaml)

# load mesh 
crazyflie_stl = load(assetpath(folder_path * "/assets/cf2_assembly.obj"))
# Load assets
floor_img = load(assetpath(folder_path * "/assets/floor/checker_repeat.png"))



end