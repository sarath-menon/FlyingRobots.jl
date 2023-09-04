function show_visualizer()

    set_theme!(backgroundcolor=:black, textcolor=:white)

    sim_time = Observable{Float64}(0.0)

    anim_state = Observable{Bool}(false)
    sim_state = Observable{Bool}(false)

    # to store plot elements
    elements = Dict()

    elements[:anim_state] = anim_state
    elements[:sim_state] = sim_state

    elements[:sim_time] = sim_time

    plot_yaml = YAML.load_file(folder_path * "/parameters/plot.yml"; dicttype=Dict{Symbol,Any})
    plot_params = recursive_dict_to_namedtuple(plot_yaml)

    elements[:configs_vec] = get_configs_vec(plot_params.graph.axis.configs)

    params = Dict(:closeup_visualizer => plot_params.closeup_visualizer,
        :fullscene_visualizer => plot_params.fullscene_visualizer,
        :plot_2d => plot_params.graph)

    elements[:params] = params

    fig = plot_empty_figure()
    elements[:fig] = fig

    # add grids
    # Top level
    g_top = fig[0, 1:2] = GridLayout()
    g_left = fig[0:1, 1] = GridLayout()
    g_right = fig[1, 2] = GridLayout()

    # Right Grid 
    g_right_plots = g_right[1, 1] = GridLayout()
    g_state_plots = g_right_plots[1, 1] = GridLayout()
    g_control_plots = g_right_plots[2, 1] = GridLayout()

    g_right_widgets = g_right[2, 1] = GridLayout()

    # Left Grid 
    g_left_plots = g_left[0:2, 1] = GridLayout()
    g_left_widgets = g_left[3, 1] = GridLayout(tellwidth=false)

    #Box(g_left_plots[1, 1], color=(:red, 0.2), strokewidth=0)
    # Box(g_left_plots[1, 1], color=(:green, 0.2), strokewidth=0)

    # # Column size adjust
    colsize!(g_right_plots, 1, Auto(0.5))

    # how much to shrink control plots grid
    rowsize!(g_right_plots, 2, Auto(0.6))
    rowsize!(g_left_widgets, 1, Auto(0.6))

    # add titles
    add_titles(elements, g_top, "Jarvis")

    # add_closeup_visualizer(elements, g_left, g_left_plots)
    add_closeup_visualizer_attitude(elements, g_left, g_left_plots)

    add_fullscene_visualizer(elements, g_left, g_left_plots)

    add_3d_model_closeup_visualizer(elements, crazyflie_stl)

    add_3d_model_fullscene_visualizer(elements, crazyflie_stl)

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

function add_closeup_visualizer_attitude(elements, g_left, g_left_plots)

    fig = elements[:fig]

    params = elements[:params][:closeup_visualizer]

    # # 3d axis for airplane visualization
    # vis_ax = Axis3(g_left_plots[1, 1],
    #     # title=params.title,
    #     limits=(params.axis.low, params.axis.high, params.axis.low, params.axis.high, params.axis.low, params.axis.high),
    #     aspect=(params.axis.aspect_x, params.axis.aspect_y, params.axis.aspect_z),
    #     halign=:left,
    #     xlabelvisible=false,
    #     ylabelvisible=false,
    #     zlabelvisible=false,
    #     xticklabelsvisible=false,
    #     yticklabelsvisible=false,
    #     zticklabelsvisible=false,
    # )

    pl = PointLight(Point3f(0), RGBf(20, 20, 20))
    al = AmbientLight(RGBf(0.2, 0.2, 0.2))

    vis_ax = LScene(g_left_plots[1, 1], show_axis=false, scenekw=(lights=[pl, al], backgroundcolor=:black, clear=true))

    cam3d!(vis_ax.scene)

    camera_height = 1.5
    camc = cameracontrols(vis_ax.scene)
    update_cam!(vis_ax.scene, camc, Vec3f(0, 0, 0), Vec3f(-4.0, camera_height, -2))

    # force 3d visualizer to have an aspect ratio of 1
    rowsize!(g_left_plots, 1, Auto(0.5))

    elements[:closeup_visualizer] = Dict{Symbol,Any}(:axis => vis_ax)
end

function add_fullscene_visualizer(elements, g_left, g_left_plots)

    fig = elements[:fig]
    # params = elements[:params]
    params = elements[:params][:fullscene_visualizer]

    # 3d axis for airplane visualization
    vis_ax = Axis3(g_left_plots[2, 1],
        # title=params.title,
        limits=(params.axis.x_low, params.axis.x_high,
            params.axis.y_low, params.axis.y_high,
            params.axis.z_low, params.axis.z_high),
        aspect=(params.axis.aspect_x, params.axis.aspect_y, params.axis.aspect_z),
        elevation=params.axis.elevation * pi,
        azimuth=params.axis.azimuth * pi,
        xlabel=params.axis.labels.x, xlabelsize=params.axis.label_size,
        ylabel=params.axis.labels.y, ylabelsize=params.axis.label_size,
        zlabel=params.axis.labels.z, zlabelsize=params.axis.label_size,
        halign=:left,
        xspinesvisible=:false,
        yspinesvisible=:false,
        zspinesvisible=:false,
        xlabeloffset=params.axis.label_offset,
        ylabeloffset=params.axis.label_offset,
        zlabeloffset=params.axis.label_offset,
        xgridwidth=params.axis.grid_width,
        ygridwidth=params.axis.grid_width,
        zgridwidth=params.axis.grid_width,
        xticklabelsvisible=:false,
        yticklabelsvisible=:false,
        zticklabelsvisible=:false,
        alignmode=Outside(-50)
    )

    # plot cube volume 
    bbox_length = params.axis.x_high - params.axis.x_low
    bbox_width = params.axis.y_high - params.axis.y_low
    bbox_height = params.axis.z_high - params.axis.z_low

    # mr = Rect3f(Vec3f(0), Vec3f(bbox_length, bbox_width, bbox_height))
    # bbox_volume = mesh!(vis_ax, mr; color=(:yellow, 0.1), transparency=true)
    # translate!(bbox_volume, Vec3f(-bbox_length / 2, -bbox_width / 2, 0))

    # # plot cube wireframe
    # bbox_wireframe = wireframe!(vis_ax, mr; color=:black, linewidth=0.4)
    # translate!(bbox_wireframe, Vec3f(-bbox_length / 2, -bbox_width / 2, 0))

    # add floor
    floor_width = 50
    floor_mesh = meshcube(Vec3f(0.5, 0.5, 0.46), Vec3f(bbox_length, bbox_width, 0.01))
    # floor = mesh!(vis_ax, floor_mesh; color=:grey, interpolate=false, diffuse=Vec3f(0.4), specular=Vec3f(0.4))
    floor = mesh!(vis_ax, floor_mesh; color=:green, interpolate=false)

    translate!(floor, Vec3f(-bbox_length / 2, -bbox_width / 2, 0))

    elements[:fullscene_visualizer] = Dict{Symbol,Any}(:axis => vis_ax)
end

function meshcube(o=Vec3f(0), sizexyz=Vec3f(1))
    uvs = map(v -> v ./ (3, 2), Vec2f[
        (0, 0), (0, 1), (1, 1), (1, 0),
        (1, 0), (1, 1), (2, 1), (2, 0),
        (2, 0), (2, 1), (3, 1), (3, 0),
        (0, 1), (0, 2), (1, 2), (1, 1),
        (1, 1), (1, 2), (2, 2), (2, 1),
        (2, 1), (2, 2), (3, 2), (3, 1),
    ])
    m = normal_mesh(Rect3f(Vec3f(-0.5) .+ o, sizexyz))
    m = GeometryBasics.Mesh(meta(coordinates(m);
            uv=uvs, normals=normals(m)), faces(m))
end

function add_3d_model_closeup_visualizer(elements, stl_file)

    params = elements[:params][:closeup_visualizer]

    vis_ax = elements[:closeup_visualizer][:axis]

    model = mesh!(vis_ax, stl_file, color=:green, shading=false)

    scale!(model, params.mesh.scale, params.mesh.scale, params.mesh.scale)

    # draw axis lines 
    ld = 0.05
    md = ld + 0.02

    line_p = [0, ld]

    # z-line
    lines!(vis_ax, zeros(2), zeros(2), line_p, color=:white, linewidth=1)
    # y-line
    lines!(vis_ax, zeros(2), line_p, zeros(2), color=:white, linewidth=1)
    # x-line
    lines!(vis_ax, line_p, zeros(2), zeros(2), color=:white, linewidth=1)

    scatter!(vis_ax, Point3f(md, 0.0, 0.0), marker='r', markersize=50, color=:white)
    scatter!(vis_ax, Point3f(0.0, md, 0.0), marker='p', markersize=50, color=:white)
    scatter!(vis_ax, Point3f(0.0, 0.0, md), marker='y', markersize=50, color=:white)

    # apply initial orientation
    rotate_mesh(model, QuatRotation(1, 0, 0, 0))

    elements[:closeup_visualizer][:model] = model
end

function add_3d_model_fullscene_visualizer(elements, stl_file)

    params = elements[:params][:fullscene_visualizer]

    vis_ax = elements[:fullscene_visualizer][:axis]

    model = mesh!(vis_ax, stl_file, color=:white, shading=false)

    scale!(model, params.mesh.scale, params.mesh.scale, params.mesh.scale)

    # center mesh at the origin
    translate!(model, Vec3f(params.mesh.initial_translation[1], params.mesh.initial_translation[2], params.mesh.initial_translation[3]))

    # apply initial orientation
    rotate_mesh(model, QuatRotation(1, 0, 0, 0))

    elements[:fullscene_visualizer][:model] = model
end


function add_2d_plots(elements, g_state_plots, g_control_plots)

    fig = elements[:fig]
    plot_2d_params = elements[:params][:plot_2d]

    plots_2d = Dict()

    state_plots = Axis[]
    control_plots = Axis[]

    for i in 1:plot_2d_params.n_state
        plot = Axis(fig,
            backgroundcolor=RGBAf(17, 34, 40, 0.1),
            xgridcolor=RGBAf(0, 200, 0, 0.5),
            ygridcolor=RGBAf(0, 200, 0, 0.5),
            xgridwidth=0.6,
            ygridwidth=0.6,
            xticklabelcolor=RGBf(220, 220, 220),
            yticklabelcolor=RGBf(220, 220, 220),)

        push!(state_plots, plot)

        # g_right_plots[i,1] = state_plots[i]
        g_state_plots[i, 1] = state_plots[i]
    end


    for i in 1:plot_2d_params.n_control
        plot = Axis(fig,
            backgroundcolor=RGBAf(17, 34, 40, 0.1),
            xgridcolor=RGBAf(0, 200, 0, 0.5),
            ygridcolor=RGBAf(0, 200, 0, 0.5),
            xgridwidth=0.6,
            ygridwidth=0.6,
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

    #buttons
    play_btn = Button(fig, label="Play", tellwidth=false, halign=:center, fontsize=30, buttoncolor=:yellow, labelcolor=:black)
    start_sim_btn = Button(fig, label="Start Sim", tellwidth=false, halign=:center, fontsize=30, buttoncolor=:yellow, labelcolor=:black)

    timeline_left_label = Label(fig, "0.0 s", justification=:left)
    timeline_right_label = Label(fig, "10.0 s", justification=:left)

    g_left_widgets[1, 1] = timeline_left_label
    g_left_widgets[1, 2] = timeline_slider
    g_left_widgets[1, 3] = timeline_right_label

    lower_left_menu = GridLayout()

    # toggle buttons
    toggles = [Toggle(fig, active=active, width=80, height=35) for active in [true]]
    labels = [Label(fig, label, fontsize=20) for label in ["Accelerated Sim"]]

    left_toggles = g_right_widgets[1, 1] = GridLayout()

    left_toggles[1, 1] = grid!(hcat(toggles[1], labels[1]), tellheight=false, tellwidth=false)
    # left_toggles[1, 2] = grid!(hcat(toggles[2], labels[2]), tellheight=false, tellwidth=false)

    lower_left_menu[1, 1] = start_sim_btn
    lower_left_menu[1, 2] = play_btn
    lower_left_menu[1, 3] = left_toggles

    g_left_widgets[2, 1:3] = lower_left_menu

    # shrink right widgets grid to make space for plots
    # rowsize!(lower_left_menu, 1, Auto(0.1))

    # attitude reset button
    attitude_reset_btn = Button(fig, label="Reset Attitude", tellwidth=false)
    g_right_widgets[1, 1] = attitude_reset_btn

    # dropdown menu
    config_menu = Menu(fig,
        options=elements[:configs_vec],
        default="positions")

    g_right_widgets[1, 2] = config_menu

    widgets = Dict()

    widgets[:timeline_slider] = timeline_slider
    widgets[:play_btn] = play_btn
    widgets[:start_sim_btn] = start_sim_btn

    widgets[:attitude_reset_btn] = attitude_reset_btn
    widgets[:config_menu] = config_menu

    elements[:widgets] = widgets
end

function plot_initialize(elements)
    state_plots = elements[:plots_2d][:state_plots]
    control_plots = elements[:plots_2d][:control_plots]

    plot_2d_params = elements[:params][:plot_2d]

    state_data = [Observable{Vector{Float64}}(zeros(1)) for i = 1:plot_2d_params.n_state]
    reference_data = [Observable{Vector{Float64}}(zeros(1)) for i = 1:plot_2d_params.n_state]

    control_data = [Observable{Vector{Float64}}(zeros(1)) for i = 1:plot_2d_params.n_control]


    time_vec = Observable{Vector{Float64}}(zeros(1))

    # Intial plot data

    for i = 1:plot_2d_params.n_state

        # state plots - state data
        lines!(state_plots[i], time_vec, state_data[i], color=:white)

        # state plots - reference data
        lines!(state_plots[i], time_vec, reference_data[i], color=:red, linestyle=:dash, linewidth=4)
    end

    for i = 1:plot_2d_params.n_control

        # control plots 
        lines!(control_plots[1], time_vec, control_data[i], color=:white)
    end

    plots_2d_data = PlotData(; time_vec=time_vec,
        state=state_data,
        reference=reference_data,
        control=control_data)

    elements[:plots_2d_data] = plots_2d_data
    # return plots_2d_data
end


function plot_reset(elements)

    plots_2d_data = elements[:plots_2d_data]

    for i = 1:3
        # set state data 
        plots_2d_data.state[i][] = [0]

        # set reference 
        plots_2d_data.reference[i][] = [0]

    end

    plots_2d_data.time_vec[] = [0]
end

function define_interactions(elements, sim_time)

    time_title = elements[:titles][:time_title]
    anim_state = elements[:anim_state]
    sim_state = elements[:sim_state]

    # to set time in title
    on(sim_time) do time
        time_title.text[] = "Time: " * string(time) * " s"
    end

    # change displayed time according to slider position
    timeline_slider = elements[:widgets][:timeline_slider]
    play_btn = elements[:widgets][:play_btn]
    start_sim_btn = elements[:widgets][:start_sim_btn]

    lift(timeline_slider.value) do val
        sim_time[] = val
    end

    # play 3d visualization if 'Play' button is clicked
    on(play_btn.clicks) do clicks

        # if sim is not already running, start sim
        if anim_state[] == false

            start_3d_animation(elements)
        else
            stop_3d_animation(elements)
        end
    end

    # start stepping sim if start_sim_btn is clicked
    on(start_sim_btn.clicks) do clicks

        Core.println("Starting simulation")

        # if sim is not already running, start sim
        if sim_state[] == false

            # start_3d_animation(elements)
            sim_state[] = true
            start_sim_btn.label = "Stop Sim"
            Core.println("Start sim button pressed")

        else
            sim_state[] = false
            start_sim_btn.label = "Start Sim"
            Core.println("Stop sim button pressed")
        end
    end
end

function set_sim_instance(elements, df::DataFrame)
    elements[:df] = df
end

# function set_sim_flag(elements, sim_state)
#     elements[:sim_state] = sim_state
# end