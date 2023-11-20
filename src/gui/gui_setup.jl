
import ..Simulation: SimAccMode, SimState, SimIdle, SimRunning
import ..Simulation: RealtimeSim, AcceleratedSim

function show_visualizer()

    set_theme!(backgroundcolor=:black, textcolor=:white)

    sim_time = Observable{Float64}(0.0)

    anim_state = Observable{Bool}(false)
    sim_cmd = Observable{SimState}(SimIdle())
    sim_acc_mode = Observable{SimAccMode}(AcceleratedSim())

    # to store plot elements
    elements = Dict()

    elements[:anim_state] = anim_state
    elements[:sim_cmd] = sim_cmd
    elements[:sim_acc_mode] = sim_acc_mode

    elements[:locks] = Dict{Symbol,Base.AbstractLock}()
    elements[:locks][:sim_channel] = Threads.SpinLock()
    elements[:locks][:recv_buffer] = Threads.SpinLock()

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
    colsize!(fig.layout, 1, Aspect(1, 1.0))

    # how much to shrink control plots grid
    rowsize!(g_right_plots, 2, Auto(0.6))
    rowsize!(g_left_widgets, 1, Auto(0.6))

    # add titles
    add_titles(elements, g_top, "Jarvis")

    # add_closeup_visualizer(elements, g_left, g_left_plots)
    add_closeup_visualizer_attitude(elements, g_left, g_left_plots)

    add_fullscene_visualizer(elements, g_left_plots)

    add_3d_model_closeup_visualizer(elements, crazyflie_stl)

    add_3d_model_fullscene_visualizer(elements, crazyflie_stl)

    add_2d_plots(elements, g_state_plots, g_control_plots)

    add_widgets(elements, g_left_widgets, g_right_widgets)

    g_right[2, 1] = g_right_widgets

    plot_initialize(elements)

    define_interactions(elements, sim_time)

    # second window setup
    # gui_second_window_setup(elements, folder_path * config_params_path)

    # define_interactions_second_window(elements)

    return elements
end


function show_visualizer_only()

    set_theme!(backgroundcolor=:black, textcolor=:white)

    sim_time = Observable{Float64}(0.0)

    anim_state = Observable{Bool}(false)
    sim_cmd = Observable{SimState}(SimIdle())
    sim_acc_mode = Observable{SimAccMode}(AcceleratedSim())

    # to store plot elements
    elements = Dict()

    elements[:anim_state] = anim_state
    elements[:sim_cmd] = sim_cmd
    elements[:sim_acc_mode] = sim_acc_mode

    elements[:locks] = Dict{Symbol,Base.AbstractLock}()
    elements[:locks][:sim_channel] = Threads.SpinLock()
    elements[:locks][:recv_buffer] = Threads.SpinLock()

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
    g_vis = fig[1, 1] = GridLayout()

    add_fullscene_visualizer(elements, g_vis; full_height=true)

    add_3d_model_fullscene_visualizer(elements, crazyflie_stl)

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

function hide_plots(elements)
    fig = elements[:fig]

    hiddenlayout_left = GridLayout(bbox=BBox(-1000, -1000, 0, 0))

    # increase 3d visualizer size 
    colsize!(fig.layout, 1, Aspect(1, 2.5))
end

function unhide_plots(elements)
    fig = elements[:fig]

    # increase 3d visualizer size 
    colsize!(fig.layout, 1, Aspect(1, 1.1))
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

function add_fullscene_visualizer(elements, g_vis; full_height=false)

    # params = elements[:params]
    params = elements[:params][:fullscene_visualizer]

    # # 3d axis for airplane visualization
    # vis_ax = Axis3(g_left_plots[2, 1],
    #     # title=params.title,
    #     limits=(params.axis.x_low, params.axis.x_high,
    #         params.axis.y_low, params.axis.y_high,
    #         params.axis.z_low, params.axis.z_high),
    #     aspect=(params.axis.aspect_x, params.axis.aspect_y, params.axis.aspect_z),
    #     elevation=params.axis.elevation * pi,
    #     azimuth=params.axis.azimuth * pi,
    #     xlabel=params.axis.labels.x, xlabelsize=params.axis.label_size,
    #     ylabel=params.axis.labels.y, ylabelsize=params.axis.label_size,
    #     zlabel=params.axis.labels.z, zlabelsize=params.axis.label_size,
    #     halign=:left,
    #     xspinesvisible=:false,
    #     yspinesvisible=:false,
    #     zspinesvisible=:false,
    #     xlabeloffset=params.axis.label_offset,
    #     ylabeloffset=params.axis.label_offset,
    #     zlabeloffset=params.axis.label_offset,
    #     xgridwidth=params.axis.grid_width,
    #     ygridwidth=params.axis.grid_width,
    #     zgridwidth=params.axis.grid_width,
    #     xticklabelsvisible=:false,
    #     yticklabelsvisible=:false,
    #     zticklabelsvisible=:false,
    #     alignmode=Outside(-50)
    # )

    if full_height == true
        grid_ = g_vis[1, 1]
    else
        grid_ = g_vis[2, 1]
    end

    lscene = LScene(grid_, show_axis=false, scenekw=(backgroundcolor=:black, clear=true))

    # now you can plot into lscene like you're used to
    root_scene = lscene.scene
    cam3d!(root_scene)

    camc = cameracontrols(root_scene)
    update_cam!(root_scene, camc, Vec3f(40, 0, 15), Vec3f(0, 0, 0))

    # g_vis[1, 1] = lscene
    vis_ax = lscene.scene

    # plot cube volume 
    bbox_length = params.axis.x_high - params.axis.x_low
    bbox_width = params.axis.y_high - params.axis.y_low
    bbox_height = params.axis.z_high - params.axis.z_low

    add_floor(vis_ax)

    elements[:fullscene_visualizer] = Dict{Symbol,Any}(:axis => vis_ax)
end

function add_floor(vis_ax)

    # add floor
    floor_width = 50
    # floor_mesh = meshcube(Vec3f(0, 0, 0), Vec3f(floor_width, floor_width, 0.01))
    floor_mesh = meshcube(Vec3f(0, 0, -0.25), Vec3f(floor_width, floor_width, 0.01), floor_width, floor_width)
    floor = mesh!(vis_ax, floor_mesh; color=floor_img, interpolate=false, diffuse=Vec3f(0.4), specular=Vec3f(0.1))
end


function meshcube(pos, sizexyz, length, width)
    uvs = map(v -> v ./ (2, 2), Vec2f[
        (0, 0), (0, 1), (1, 1), (1, 0),
        (1, 0), (1, 1), (2, 1), (2, 0),
        (2, 0), (2, 1), (3, 1), (3, 0),
        (0, 1), (0, 2), (1, 2), (1, 1),
        (1, 1), (1, 2), (2, 2), (2, 1),
        (2, 1), (2, 2), (3, 2), (3, 1),
    ])
    m = normal_mesh(Rect3f(Vec3f(-length / 2, -width / 2, 0) + pos, sizexyz))
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

    model = mesh!(vis_ax, stl_file, color=:blue, shading=false)

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
    open_menu_btn = Button(fig, label="Open menu", tellwidth=false, halign=:center, fontsize=30, buttoncolor=:yellow, labelcolor=:black)

    timeline_left_label = Label(fig, "0.0 s", justification=:left)
    timeline_right_label = Label(fig, "10.0 s", justification=:left)

    g_left_widgets[1, 1] = timeline_left_label
    g_left_widgets[1, 2] = timeline_slider
    g_left_widgets[1, 3] = timeline_right_label

    lower_left_menu = GridLayout()

    # toggle buttons
    sim_acc_toggle = Toggle(fig, active=true, width=80, height=35)
    sim_acc_toggle_label = Label(fig, text="Accelerated Sim", fontsize=25)

    sim_acc_toggle_grid = grid!(hcat(sim_acc_toggle, sim_acc_toggle_label), tellheight=false, tellwidth=false)
    # left_toggles[1, 2] = grid!(hcat(toggles[2], labels[2]), tellheight=false, tellwidth=false)

    lower_left_menu[1, 1] = start_sim_btn
    lower_left_menu[1, 2] = play_btn
    lower_left_menu[1, 3] = sim_acc_toggle_grid
    lower_left_menu[1, 4] = open_menu_btn

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
    widgets[:open_menu_btn] = open_menu_btn

    widgets[:sim_acc_toggle] = sim_acc_toggle
    widgets[:sim_acc_toggle_label] = sim_acc_toggle_label

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


function set_sim_instance(elements, df::DataFrame)
    elements[:df] = df
end

# function set_sim_flag(elements, sim_cmd)
#     elements[:sim_cmd] = sim_cmd
# end

function open_second_window()
    set_theme!(backgroundcolor=:white, textcolor=:black, fontsize=25)

    fig = Figure(resolution=(1920, 1080))

    g_left = fig[1, 1] = GridLayout(valign=:top, halign=:left)

    g_planner = fig[1, 2] = GridLayout(valign=:top, halign=:left)
    g_controller = fig[1, 2] = GridLayout(valign=:top, halign=:left)

    # hidden layouts
    hiddenlayout_left = GridLayout(bbox=BBox(-1000, -1000, 0, 0))

    # Right Grids
    g_left_menu = g_left[1, 1] = GridLayout(halign=:left, valign=:top)

    # Left menu -----------------------------------------------------------------------------

    buttongrid = g_left_menu[1, 1] = GridLayout(tellwidth=false, halign=:left, valign=:top)
    colsize!(fig.layout, 1, Aspect(1, 0.55))
    # colsize!(g_left, 1,  Relative(1/3))

    buttonlabels = ["Planner", "controller", "estimator", "simulation"]
    buttons = buttongrid[1:length(buttonlabels), 1] = [Button(fig, label=l, halign=:left, width=200, height=80) for l in buttonlabels]

    # Planner --------------------------------------------------------
    menu1 = Menu(fig, options=["Polynomial", "Dubin's"], tellwidth=false)
    menu2 = Menu(fig, options=["Circle", "Lemniscate"], tellwidth=false)

    g_planner[1, 1] = hgrid!(
        Label(fig, "Trajectory generator: "), menu1)

    g_planner[2, 1] = hgrid!(
        Label(fig, "Type : "), menu2)


    button1 = Button(g_planner[3, 1], label="Open file", tellwidth=false)
    button2 = Button(g_planner[4, 1], label="Open folder", tellwidth=false)

    # Controller --------------------------------------------------------

    menu1 = Menu(fig, options=["Cascaded (position, attitude, attitude rate) ",
            "Cascaded (position, attitude ",
            "Monolithic"],
        tellwidth=false)

    menu2 = Menu(fig, options=["PID", "LQR"], tellwidth=false)

    menu3 = Menu(fig, options=["PID", "LQR"], tellwidth=false)

    menu4 = Menu(fig, options=["PID", "LQR"], tellwidth=false)

    g_controller[1, 1] = hgrid!(
        Label(fig, "Controller Architecture: "), menu1)

    g_controller[2, 1] = hgrid!(
        Label(fig, "Position Controller : "), menu2)

    g_controller[3, 1] = hgrid!(
        Label(fig, "Attitude Controller : ", halign=:center), menu3)

    g_controller[4, 1] = hgrid!(
        Label(fig, "Attitude rate Controller : "), menu4)


    button1 = Button(g_controller[5, 1], label="Open file", tellwidth=false)
    button2 = Button(g_controller[5, 2], label="Open folder", tellwidth=false)
    #  --------------------------------------------------------

    hiddenlayout_left[1, 1] = g_controller

    on(button1.clicks) do n
        @async begin
            f = pick_file()
            @show f
        end

    end

    on(button2.clicks) do n
        @async begin
            f = pick_folder()
            @show f
        end
    end

    for i in 1:length(buttonlabels)
        on(buttons[i].clicks) do n
            if i == 1
                fig[1, 2] = g_planner
                hiddenlayout_left[1, 1] = g_controller
            elseif i == 2
                fig[1, 2] = g_controller
                hiddenlayout_left[1, 1] = g_planner
            end
        end
    end

    # on(events(fig).window_area) do event
    #     resize_to_layout!(fig)
    # end
    # resize_to_layout!(fig)

    # ---------------------------------------------
    # screen properties   
    flag = @isdefined screen2
    if flag == true
        GLMakie.destroy!(screen2)
    end

    screen2 = GLMakie.Screen()
    display(screen2, fig)
    GLMakie.set_screen_visibility!(screen2, true)

end