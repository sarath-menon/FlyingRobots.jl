function show_visualizer()
    fig = plot_empty_figure()

    # add grids
    # Top level
    g_top = fig[0, 1:2] = GridLayout()
    g_planner = fig[1, 1] = GridLayout(alignmode=Outside(50))
    g_controller = fig[1, 2] = GridLayout()

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

    # how much to shrink control plots grid
    rowsize!(g_controller_plots, 2, Auto(0.6))

    # add titles
    elements[:super_title], elements[:time_title] = add_title(g_top, "Jarvis", sim_time_obs)

    elements[:visualizer_3d] = add_3d_visualizer(fig, vis_params, g_planner, g_planner_plots)

    elements[:model] = add_3d_model(elements[:visualizer_3d], crazyflie_stl, vis_params)

    elements[:state_plots], elements[:control_plots] = add_2d_plots(fig, g_state_plots, g_control_plots)

    timeline_slider, timeline_btn, attitude_reset_btn, config_menu = add_ui_elements(fig, g_planner_widgets, g_controller_widgets)

    g_controller[2, 1] = g_controller_widgets

    plot_data = plot_initialize(elements[:state_plots], elements[:control_plots])

    define_interactions()

    return elements, plot_data
end


function plot_empty_figure()
    fig = Figure(resolution=get_primary_resolution(1))
    display(fig)

    return fig
end

function add_title(g_top, title, sim_time_obs)
    super_title = Label(g_top[1, 1:2], title, fontsize=60)
    time_title = Label(g_top[2, 1:2], "Time = " * string(sim_time_obs[]) * " s", fontsize=40)

    return super_title, time_title
end

function add_3d_visualizer(fig, vis_params, g_planner, g_planner_plots)

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

    return vis_ax
end

function add_3d_model(vis_ax, stl_file, vis_params)

    model = mesh!(vis_ax, stl_file, color=:red)

    scale!(model, vis_params.mesh.scale, vis_params.mesh.scale, vis_params.mesh.scale)

    # center mesh at the origin
    translate!(model, Vec3f(vis_params.mesh.initial_translation[1], vis_params.mesh.initial_translation[2], vis_params.mesh.initial_translation[3]))

    # apply initial orientation
    rotate_mesh(model, QuatRotation(1, 0, 0, 0))

    return model
end


function add_2d_plots(fig, g_state_plots, g_control_plots)

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

    return state_plots, control_plots

end


function add_ui_elements(fig, g_planner_widgets, g_controller_widgets)
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

    # # toggle buttons
    # toggles = [Toggle(fig, active=active) for active in [true, true, true]]
    # labels = [Label(fig, label) for label in ["y", "z", "θ"]]

    # g_controller_toggles = g_controller_widgets[1, 3] = GridLayout()

    # g_controller_toggles[1, 1] = grid!(hcat(toggles[1], labels[1]), tellheight=false, tellwidth=false)
    # g_controller_toggles[1, 2] = grid!(hcat(toggles[2], labels[2]), tellheight=false, tellwidth=false)
    # g_controller_toggles[1, 3] = grid!(hcat(toggles[3], labels[3]), tellheight=false, tellwidth=false)

    return timeline_slider, timeline_btn, attitude_reset_btn, config_menu

end

function plot_initialize(state_plots, control_plots)
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

    plot_data = PlotData(time_vec, data_1, data_2, data_3, data_4, data_5)

    return plot_data
end


function plot_reset(plot_data)
    plot_data.axis_1[] = [0]
    plot_data.axis_2[] = [0]
    plot_data.axis_3[] = [0]

    plot_data.axis_4[] = [0]
    plot_data.axis_5[] = [0]

    plot_data.time_vec[] = [0]
end

function define_interactions()
    # to change time in title
    on(sim_time) do time
        elements[:time_title].text = "Time: " * string(time) * " s"
    end
end

