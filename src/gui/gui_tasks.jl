function show_visualizer()

    sim_time = Observable{Float64}(0.0)
    sim_state = Observable{Bool}(false)

    # to store plot elements
    elements = Dict()

    elements[:sim_state] = sim_state
    elements[:sim_time] = sim_time

    plot_yaml = YAML.load_file(folder_path * "/parameters/plot.yml"; dicttype=Dict{Symbol,Any})
    plot_params = recursive_dict_to_namedtuple(plot_yaml)

    elements[:configs_vec] = get_configs_vec(plot_params.graph.axis.configs)

    params = Dict(:visualizer_3d => plot_params.visualizer, :plot_2d => plot_params.graph)

    elements[:params] = params

    fig = plot_empty_figure()
    elements[:fig] = fig

    # add grids
    # Top level
    g_top = fig[0, 1:2] = GridLayout()
    g_left = fig[0:1, 1] = GridLayout(alignmode=Outside(50))
    g_right = fig[1, 2] = GridLayout()

    # Right Grid 
    g_right_plots = g_right[1, 1] = GridLayout()
    g_state_plots = g_right_plots[1, 1] = GridLayout()
    g_control_plots = g_right_plots[2, 1] = GridLayout()

    g_right_widgets = g_right[2, 1] = GridLayout()

    # Left Grid 
    g_left_plots = g_left[0:2, 1] = GridLayout()
    g_left_widgets = g_left[3, 1] = GridLayout(tellwidth=false)

    # Box(g_left_plots[0, 1], color=(:red, 0.2), strokewidth=0)
    # Box(g_left_plots[1, 1], color=(:green, 0.2), strokewidth=0)
    rowgap!(g_left_plots, 1)

    # # Column size adjust
    colsize!(g_left, 1, Auto(1))
    colsize!(g_right, 1, Auto(1))

    # how much to shrink control plots grid
    rowsize!(g_right_plots, 2, Auto(0.6))
    rowsize!(g_left_widgets, 1, Auto(0.6))

    # add titles
    add_titles(elements, g_top, "Jarvis")

    add_3d_visualizer(elements, g_left, g_left_plots)

    add_3d_visualizer_2(elements, g_left, g_left_plots)

    add_3d_model(elements, crazyflie_stl)

    add_2d_plots(elements, g_state_plots, g_control_plots)

    add_widgets(elements, g_left_widgets, g_right_widgets)

    g_right[2, 1] = g_right_widgets

    plot_initialize(elements)

    define_interactions(elements, sim_time)

    return elements
end


function plot_empty_figure()
    fig = Figure(resolution=get_primary_resolution(1))
    display(fig)

    return fig
end

function add_titles(elements, g_top, title)

    titles = Dict()

    titles[:super_title] = Label(g_top[0, 1], title, fontsize=60)
    titles[:time_title] = Label(g_top[1, 1], "Time = " * string(0.0) * " s", fontsize=40)

    elements[:titles] = titles
end

function add_3d_visualizer(elements, g_left, g_left_plots)

    fig = elements[:fig]
    # vis_3d_params = elements[:vis_3d_params]
    vis_3d_params = elements[:params][:visualizer_3d]

    # 3d axis for airplane visualization
    vis_ax = Axis3(g_left_plots[1, 1],
        # title=vis_3d_params.title,
        limits=(vis_3d_params.axis.low, vis_3d_params.axis.high, vis_3d_params.axis.low, vis_3d_params.axis.high, vis_3d_params.axis.low, vis_3d_params.axis.high),
        aspect=(vis_3d_params.axis.aspect_x, vis_3d_params.axis.aspect_y, vis_3d_params.axis.aspect_z),
        xlabel=vis_3d_params.axis.labels.x, xlabelsize=vis_3d_params.axis.label_size,
        ylabel=vis_3d_params.axis.labels.y, ylabelsize=vis_3d_params.axis.label_size,
        zlabel=vis_3d_params.axis.labels.z, zlabelsize=vis_3d_params.axis.label_size,
        halign=:left,
        xspinecolor_1=:white,
        xspinecolor_3=:white,
        yspinecolor_1=:white,
        yspinecolor_3=:white,
        zspinecolor_1=:white,
        zspinecolor_3=:white,
        xspinewidth=vis_3d_params.axis.spine_width,
        yspinewidth=vis_3d_params.axis.spine_width,
        zspinewidth=vis_3d_params.axis.spine_width,
        xlabeloffset=vis_3d_params.axis.label_offset,
        ylabeloffset=vis_3d_params.axis.label_offset,
        zlabeloffset=vis_3d_params.axis.label_offset,
        xgridwidth=vis_3d_params.axis.grid_width,
        ygridwidth=vis_3d_params.axis.grid_width,
        zgridwidth=vis_3d_params.axis.grid_width,
        xtickwidth=0.1,
        ytickwidth=0.1,
        ztickwidth=0.1,
    )

    # force 3d visualizer to have an aspect ratio of 1
    rowsize!(g_left_plots, 1, Auto(1.0))

    elements[:visualizer_3d] = Dict{Symbol,Any}(:axis => vis_ax)
end

function add_3d_visualizer_2(elements, g_left, g_left_plots)

    fig = elements[:fig]
    # vis_3d_params = elements[:vis_3d_params]
    vis_3d_params = elements[:params][:visualizer_3d]

    # 3d axis for airplane visualization
    vis_ax = Axis3(g_left_plots[2, 1],
        # title=vis_3d_params.title,
        limits=(0, 10, 0, 20, 0, 2),
        aspect=(1, 2, 0.2),
        elevation=0.15 * pi,
        azimuth=0.2 * pi,
        xlabel=vis_3d_params.axis.labels.x, xlabelsize=vis_3d_params.axis.label_size,
        ylabel=vis_3d_params.axis.labels.y, ylabelsize=vis_3d_params.axis.label_size,
        zlabel=vis_3d_params.axis.labels.z, zlabelsize=vis_3d_params.axis.label_size,
        halign=:left,
        xspinecolor_1=:white,
        xspinecolor_3=:white,
        yspinecolor_1=:white,
        yspinecolor_3=:white,
        zspinecolor_1=:white,
        zspinecolor_3=:white,
        xspinewidth=vis_3d_params.axis.spine_width,
        yspinewidth=vis_3d_params.axis.spine_width,
        zspinewidth=vis_3d_params.axis.spine_width,
        xlabeloffset=vis_3d_params.axis.label_offset,
        ylabeloffset=vis_3d_params.axis.label_offset,
        zlabeloffset=vis_3d_params.axis.label_offset,
        xgridwidth=vis_3d_params.axis.grid_width,
        ygridwidth=vis_3d_params.axis.grid_width,
        zgridwidth=vis_3d_params.axis.grid_width,
    )

    # force 3d visualizer to have an aspect ratio of 1
    # rowsize!(g_left, 1, Aspect(1, 1.0))

    elements[:visualizer_3d_2] = Dict{Symbol,Any}(:axis => vis_ax)
end

function add_3d_model(elements, stl_file)

    # vis_3d_params = elements[:vis_3d_params]
    vis_3d_params = elements[:params][:visualizer_3d]
    vis_ax = elements[:visualizer_3d][:axis]

    model = mesh!(vis_ax, stl_file, color=:red)

    scale!(model, vis_3d_params.mesh.scale, vis_3d_params.mesh.scale, vis_3d_params.mesh.scale)

    # center mesh at the origin
    translate!(model, Vec3f(vis_3d_params.mesh.initial_translation[1], vis_3d_params.mesh.initial_translation[2], vis_3d_params.mesh.initial_translation[3]))

    # apply initial orientation
    rotate_mesh(model, QuatRotation(1, 0, 0, 0))

    elements[:visualizer_3d][:model] = model
end


function add_2d_plots(elements, g_state_plots, g_control_plots)

    fig = elements[:fig]
    plot_2d_params = elements[:params][:plot_2d]


    plots_2d = Dict()

    state_plots = Axis[]
    control_plots = Axis[]

    for i in 1:plot_2d_params.n_state
        plot = Axis(fig)
        push!(state_plots, plot)

        # g_right_plots[i,1] = state_plots[i]
        g_state_plots[i, 1] = state_plots[i]
    end


    for i in 1:plot_2d_params.n_control
        plot = Axis(
            fig,
            # ylabel=plot_2d_params.ylabels[i] 
        )
        push!(control_plots, plot)

        # g_right_plots[i,1] = state_plots[i]
        g_control_plots[i, 1] = control_plots[i]
    end

    plots_2d[:state_plots] = state_plots
    plots_2d[:control_plots] = control_plots

    elements[:plots_2d] = plots_2d
end



function add_widgets(elements, g_left_widgets, g_right_widgets)

    fig = elements[:fig]

    # slider grid for timeline control
    timeline_slider = Slider(fig, range=0:0.01:10, startvalue=0, linewidth=25.0, tellheight=false,
        halign=:left)

    #timeline button
    timeline_btn = Button(fig, label="Play", tellwidth=false, halign=:center, fontsize=40)
    timeline_left_label = Label(fig, "0.0 s", justification=:left)
    timeline_right_label = Label(fig, "10.0 s", justification=:left)

    g_left_widgets[1, 1] = timeline_left_label
    g_left_widgets[1, 2] = timeline_slider
    g_left_widgets[1, 3] = timeline_right_label

    g_left_widgets[2, :] = timeline_btn

    # shrink right widgets grid to make space for plots
    # rowsize!(g_right, 2,  Auto(0.2))

    # attitude reset button
    attitude_reset_btn = Button(fig, label="Reset Attitude", tellwidth=false)
    g_right_widgets[1, 1] = attitude_reset_btn

    # dropdown menu
    config_menu = Menu(fig,
        options=elements[:configs_vec],
        default="positions")

    g_right_widgets[1, 2] = config_menu

    widgets = Dict()

    # # toggle buttons
    # toggles = [Toggle(fig, active=active) for active in [true, true, true]]
    # labels = [Label(fig, label) for label in ["y", "z", "θ"]]

    # g_right_toggles = g_right_widgets[1, 3] = GridLayout()

    # g_right_toggles[1, 1] = grid!(hcat(toggles[1], labels[1]), tellheight=false, tellwidth=false)
    # g_right_toggles[1, 2] = grid!(hcat(toggles[2], labels[2]), tellheight=false, tellwidth=false)
    # g_right_toggles[1, 3] = grid!(hcat(toggles[3], labels[3]), tellheight=false, tellwidth=false)

    widgets[:timeline_slider] = timeline_slider
    widgets[:timeline_btn] = timeline_btn
    widgets[:attitude_reset_btn] = attitude_reset_btn
    widgets[:config_menu] = config_menu

    elements[:widgets] = widgets
end

function plot_initialize(elements)
    state_plots = elements[:plots_2d][:state_plots]
    control_plots = elements[:plots_2d][:control_plots]

    data_1 = Observable{Vector{Float64}}(zeros(1))
    data_2 = Observable{Vector{Float64}}(zeros(1))
    data_3 = Observable{Vector{Float64}}(zeros(1))
    data_4 = Observable{Vector{Float64}}(zeros(1))
    data_5 = Observable{Vector{Float64}}(zeros(1))

    time_vec = Observable{Vector{Float64}}(zeros(1))

    # Intial plot data 
    # Plot initial data (zeros)
    lines!(state_plots[1], time_vec, data_1, color=:black)
    lines!(state_plots[2], time_vec, data_2, color=:black)
    lines!(state_plots[3], time_vec, data_3, color=:black)

    lines!(control_plots[1], time_vec, data_4, color=:black)
    lines!(control_plots[2], time_vec, data_5, color=:black)

    plots_2d_data = PlotData(time_vec, data_1, data_2, data_3, data_4, data_5)

    elements[:plots_2d_data] = plots_2d_data
    # return plots_2d_data
end


function plot_reset(plots_2d_data)
    plots_2d_data.axis_1[] = [0]
    plots_2d_data.axis_2[] = [0]
    plots_2d_data.axis_3[] = [0]

    plots_2d_data.axis_4[] = [0]
    plots_2d_data.axis_5[] = [0]

    plots_2d_data.time_vec[] = [0]
end

function define_interactions(elements, sim_time)

    time_title = elements[:titles][:time_title]
    sim_state = elements[:sim_state]

    # to set time in title
    on(sim_time) do time
        time_title.text[] = "Time: " * string(time) * " s"
    end

    # change displayed time according to slider position
    timeline_slider = elements[:widgets][:timeline_slider]
    timeline_btn = elements[:widgets][:timeline_btn]

    lift(timeline_slider.value) do val
        sim_time[] = val
    end

    # play 3d visualization if 'Play' button is clicked
    on(timeline_btn.clicks) do clicks

        # if sim is not already running, start sim
        if sim_state[] == false

            start_3d_animation(elements)
        else
            stop_3d_animation(elements)
        end
    end
end


function stop_3d_animation(elements)
    sim_state = elements[:sim_state]
    timeline_btn = elements[:widgets][:timeline_btn]

    # if sim is currently running, set sim state flag to false
    sim_state[] = false

    # change button text to show "Stop"
    timeline_btn.label = "Play"
end


function set_sim_instance(elements, df::DataFrame)
    elements[:df] = df
end

