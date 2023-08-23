
module Gui

using GLMakie
using GeometryBasics
using Rotations
using FileIO
using YAML
using FlyingRobots

include("gui_helper.jl")

set_theme!(
    #     font = "Arial", # inherited by layoutables if not overridden
    fontsize=25, # inherited by layoutables if not overridden
)

# read settings file 
folder_path = pwd() * "/examples/quad_3d"

plot_yaml = YAML.load_file(folder_path * "/parameters/plot.yml"; dicttype=Dict{Symbol,Any})
sim_yaml = YAML.load_file(folder_path * "/parameters/sim.yml"; dicttype=Dict{Symbol,Any})

sim_params = recursive_dict_to_namedtuple(sim_yaml)
plot_params = recursive_dict_to_namedtuple(plot_yaml)

# load mesh 
crazyflie_stl = load(assetpath(folder_path * "/assets/cf2_assembly.obj"))

# display empty figure
fig = Figure(resolution=get_primary_resolution(1))
display(fig)

# clear figure 
empty!(fig)

# Params --------------------------------------------

vis_params = plot_params.visualizer
graph_params = plot_params.graph

configs_vec = get_configs_vec(plot_params.graph.axis.configs)


# Observables -------------------------------------------------
sim_time_obs = Observable{Float64}(0.0)
sim_state_obs = Observable{Bool}(false)

# Top level
g_top = fig[0, 1:2] = GridLayout()
g_planner = fig[1, 1] = GridLayout(alignmode=Outside(50))
g_controller = fig[1, 2] = GridLayout()

# Add title
supertitle1 = Label(g_top[1, 1:2], plot_params.title.name, fontsize=60)
time_title = Label(g_top[2, 1:2], "Time = " * string(sim_time_obs[]) * " s", fontsize=40)

# Box(fig[0, 1:2], color = (:red, 0.2), strokewidth = 0)

# Right Grid 
g_controller_plots = g_controller[1, 1] = GridLayout()
g_state_plots = g_controller_plots[1, 1] = GridLayout()
g_control_plots = g_controller_plots[2, 1] = GridLayout()

g_controller_widgets = g_controller[2, 1] = GridLayout()

# Left Grid 
g_planner_plots = g_planner[1, 1] = GridLayout()
g_planner_widgets = g_planner[2, 1] = GridLayout(tellwidth=false)


# # Column size adjust
colsize!(g_planner, 1, Auto(1))
colsize!(g_controller, 1, Auto(1))

# 3d axis for airplane visualization
vis_ax = Axis3(g_planner_plots[1, 1],
    title=vis_params.title,
    limits=(vis_params.axis.low, vis_params.axis.high, vis_params.axis.low, vis_params.axis.high, vis_params.axis.low, vis_params.axis.high),
    aspect=(vis_params.axis.aspect_x, vis_params.axis.aspect_y, vis_params.axis.aspect_z),
    xlabel=vis_params.axis.labels.x, xlabelsize=vis_params.axis.label_size,
    ylabel=vis_params.axis.labels.y, ylabelsize=vis_params.axis.label_size,
    zlabel=vis_params.axis.labels.z, zlabelsize=vis_params.axis.label_size,
    halign=:left,
    xspinecolor_1=:white,
    xspinecolor_3=:white,
    yspinecolor_1=:white,
    yspinecolor_3=:white,
    zspinecolor_1=:white,
    zspinecolor_3=:white,
    xspinewidth=vis_params.axis.spine_width,
    yspinewidth=vis_params.axis.spine_width,
    zspinewidth=vis_params.axis.spine_width,
    xlabeloffset=vis_params.axis.label_offset,
    ylabeloffset=vis_params.axis.label_offset,
    zlabeloffset=vis_params.axis.label_offset,
    xgridwidth=vis_params.axis.grid_width,
    ygridwidth=vis_params.axis.grid_width,
    zgridwidth=vis_params.axis.grid_width,
)

# force 3d visualizer to have an aspect ratio of 1
rowsize!(g_planner, 1, Aspect(1, 1.0))

m = mesh!(vis_ax, crazyflie_stl, color=:red)

scale!(m, vis_params.mesh.scale, vis_params.mesh.scale, vis_params.mesh.scale)

# center mesh at the origin
translate!(m, Vec3f(vis_params.mesh.initial_translation[1], vis_params.mesh.initial_translation[2], vis_params.mesh.initial_translation[3]))

# apply initial orientation
rotate_mesh(m, to_std_quaternion(sim_params.vehicle.initial_state.q_BI))

# Axes for 2D plots ------------------------------------------------

state_plots = Axis[]
control_plots = Axis[]

for i in 1:graph_params.n_state
    plot = Axis(fig, ylabel=graph_params.ylabels[i], titlesize=graph_params.titlesize)
    push!(state_plots, plot)

    # g_controller_plots[i,1] = state_plots[i]
    g_state_plots[i, 1] = state_plots[i]
end


for i in 1:graph_params.n_control
    plot = Axis(
        fig,
        # ylabel=graph_params.ylabels[i] 
    )
    push!(control_plots, plot)

    # g_controller_plots[i,1] = state_plots[i]
    g_control_plots[i, 1] = control_plots[i]
end

# UI elements ------------------------------------------------

# slider grid for timeline control
timeline_slider = Slider(fig, range=0:0.01:10, startvalue=0, linewidth=25.0, tellheight=false,
    halign=:left)

#timeline button
timeline_btn = Button(fig, label="Play", tellwidth=false, halign=:center, fontsize=40)
timeline_left_label = Label(fig, "0.0 s", justification=:left)
timeline_right_label = Label(fig, "10.0 s", justification=:left)

g_planner_widgets[1, 1] = timeline_left_label
g_planner_widgets[1, 2] = timeline_slider
g_planner_widgets[1, 3] = timeline_right_label

g_planner_widgets[2, :] = timeline_btn


# how much to shrink control plots grid
rowsize!(g_controller_plots, 2, Auto(0.2))

# shrink right widgets grid to make space for plots
# rowsize!(g_controller, 2,  Auto(0.2))

# attitude reset button
attitude_reset_btn = Button(fig, label="Reset Attitude", tellwidth=false)
g_controller_widgets[1, 1] = attitude_reset_btn

# dropdown menu
config_menu = Menu(fig,
    options=configs_vec,
    default="positions")

g_controller_widgets[1, 2] = config_menu

# toggle buttons
toggles = [Toggle(fig, active=active) for active in [true, true, true]]
labels = [Label(fig, label) for label in ["y", "z", "θ"]]

g_controller_toggles = g_controller_widgets[1, 3] = GridLayout()

g_controller_toggles[1, 1] = grid!(hcat(toggles[1], labels[1]), tellheight=false, tellwidth=false)
g_controller_toggles[1, 2] = grid!(hcat(toggles[2], labels[2]), tellheight=false, tellwidth=false)
g_controller_toggles[1, 3] = grid!(hcat(toggles[3], labels[3]), tellheight=false, tellwidth=false)

g_controller[2, 1] = g_controller_widgets


# event handling
on(config_menu.selection) do config
    plot_trajectory(configs_vec, config)
end

on(timeline_btn.clicks) do clicks

    # if sim is not already running, start sim
    if sim_state_obs[] == false

        # set sim state flag to true
        sim_state_obs[] = true

        # change button text to show "Stop"
        timeline_btn.label = "Stop"

        # start sim task asynchronously
        sim_task = @async plot_3d_trajectory(duration=3.0, sim_time_obs=sim_time_obs, sim_state_obs=sim_state_obs)
        #plot_3d_trajectory(duration=3.0, sim_time_obs=sim_time_obs, sim_state_obs=sim_state_obs)

        @async begin
            # wait for 3d trajectory plotting to finish
            wait(sim_task)

            # set sim state flag to false 
            sim_state_obs[] = false

            # change button text to show "Play"
            timeline_btn.label = "Play"
        end

    else
        # if sim is currently running, set sim state flag to false
        sim_state_obs[] = false
    end

end


# lift(timeline_slider.value) do val
#     # sim_time_obs[] = val
# end

on(sim_time_obs) do time
    time_title.text = "Time: " * string(time) * " s"
end

time_marker = lift(sim_time_obs) do time_val
    time_val
end

mutable struct PlotData3
    time_vec::Observable{Vector{Float64}}

    axis_1::Observable{Vector{Float64}}
    axis_2::Observable{Vector{Float64}}
    axis_3::Observable{Vector{Float64}}
end

PlotData = PlotData3


function plot_initialize(state_plots)
    data_1 = Observable{Vector{Float64}}(zeros(1))
    data_2 = Observable{Vector{Float64}}(zeros(1))
    data_3 = Observable{Vector{Float64}}(zeros(1))

    time_vec = Observable{Vector{Float64}}(zeros(1))

    # Intial plot data 
    # Plot initial data (zeros)
    lines!(state_plots[1], time_vec, data_1)
    lines!(state_plots[2], time_vec, data_2)
    lines!(state_plots[3], time_vec, data_3)

    plot_data = PlotData(time_vec, data_1, data_2, data_3)

    return plot_data
end



plot_data = plot_initialize(state_plots)


function plot_reset()
    plot_data.axis_1[] = [0]
    plot_data.axis_2[] = [0]
    plot_data.axis_3[] = [0]

    plot_data.time_vec[] = [0]
end






# initial setup
# plot_trajectory(config_dict, config_menu.selection[])
# plot_trajectory(configs_vec, config_menu.selection[])

# traj_count_n = 1000
# lines!(vis_ax, zeros(traj_count_n), df.y_req[1:traj_count_n], df.z_req[1:traj_count_n], linestyle=:dash)

# trim!(f.layout)

# display(f)
end