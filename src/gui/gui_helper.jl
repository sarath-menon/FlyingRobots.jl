# # convert standrard quaternion, euler angle to makie quaternin type
function to_makie_quaternion(q)::Quaternionf
    return Quaternionf(q.q.s, q.q.v1, q.q.v2, q.q.v3)
end

function to_std_quaternion(q)
    return QuatRotation(q.q_w, q.q_x, q.q_y, q.q_z)
end

function rotate_mesh(m, q)
    makie_q = to_makie_quaternion(q)
    GLMakie.rotate!(m, makie_q)
end

function get_primary_resolution(index::Integer)
    monitors = GLMakie.GLFW.GetMonitors()

    videomode = GLMakie.MonitorProperties(monitors[index]).videomode
    (xscale, yscale) = GLMakie.GLFW.GetMonitorContentScale(monitors[index])

    (height, width) = (videomode.width * xscale, videomode.height * yscale)
    (height, width) = (convert(Int, height), convert(Int, width))
    return (height, width)
end

function plot_axis2d(axis::Axis; x::Vector{Float64}, y::Vector{Float64}, title::String, linestyle=nothing, color=:black, linewidth=3)
    # set title
    axis.title = title

    # fraw plot
    lines!(axis, x, y, linestyle=linestyle, color=color, linewidth=linewidth)
end

function plot_axis2d_tm(axis::Axis; x::Observable{Float64}, y::Observable{Float64}, color=:black)
    # fraw plot
    vlines!(axis, x, color=color, linewidth=2)
end

function plot_trajectory(config_dict::Dict, config)
    # diplayed plots
    for (i, title) in enumerate(config_dict[config])
        # clear axis
        empty!(state_plots[i])

        # plot actual trajectory 
        plot_axis2d(state_plots[i]; x=df.timestamp, y=df[!, title], title=title, linewidth=2)

        # plot desired trajectory 
        plot_axis2d(state_plots[i]; x=df.timestamp, y=df[!, title*"_req"], title=title, linestyle=:dash, color=:green, linewidth=6)

        # plot time marker
        plot_axis2d_tm(state_plots[i]; x=time_marker, y=time_marker)

    end
end

function plot_trajectory(configs_vec::Vector{String}, config::String)

    titles_vec = get_axis_titles(configs_vec, config)


    # diplayed plots
    for (i, title) in enumerate(titles_vec)
        # clear axis
        empty!(state_plots[i])

        # plot actual trajectory 
        plot_axis2d(state_plots[i]; x=df.timestamp, y=df[!, title], title=title, linewidth=2)

        # plot desired trajectory 
        plot_axis2d(state_plots[i]; x=df.timestamp, y=df[!, title*"_req"], title=title, linestyle=:dash, color=:green, linewidth=6)

        # plot time marker
        plot_axis2d_tm(state_plots[i]; x=time_marker, y=time_marker)

    end
end


# function plot_3d_trajectory(x_pos, y_pos, z_pos; sim_time_obs::Observable, sim_state_obs::Observable, duration=10.0, dt=0.01, frame_rate=25)

function plot_3d_trajectory(df, elements; duration=10.0, dt=0.01, frame_rate=25)

    sim_state = elements[:sim_state]
    sim_time = elements[:sim_time]

    timeline_slider = elements[:widgets][:timeline_slider]

    step_count = convert(Integer, duration / dt)
    n_skip_frames = convert(Integer, 100 / frame_rate)

    @show sim_state

    # check if simulation is already running
    if sim_state[] == true
        return
    end

    sim_state[] = true

    for i in 1:n_skip_frames:step_count

        # stop simulation is stop button is pressed
        if sim_state[] == false
            break
        end

        position = Vec3d(df[!, 2][i], df[!, 3][i], df[!, 4][i])
        orientation = QuatRotation(df[!, 8][i], df[!, 9][i], df[!, 10][i], df[!, 11][i])

        model_set_pose(elements, position, orientation)

        # set the time observable
        sim_time[] = round(df[!, "timestamp"][i], digits=2)

        # set timeline slider value 
        set_close_to!(timeline_slider, sim_time[])

        # account for skipped frames in sleep time
        sleep(n_skip_frames * dt)

    end
end


struct PlotData5
    time_vec::Observable{Vector{Float64}}

    axis_1::Observable{Vector{Float64}}
    axis_2::Observable{Vector{Float64}}
    axis_3::Observable{Vector{Float64}}

    axis_4::Observable{Vector{Float64}}
    axis_5::Observable{Vector{Float64}}
end

PlotData = PlotData5

function model_set_pose(elements, position, orientation)

    plot_params = elements[:plot_params]

    max_range = plot_params.visualizer.axis.high - plot_params.visualizer.axis.low
    dist = max_range / 2

    x_low = position.x - dist
    y_low = position.y - dist
    z_low = position.z - dist

    x_high = position.x + dist
    y_high = position.y + dist
    z_high = position.z + dist

    elements[:visualizer_3d].limits = (x_low, x_high, y_low, y_high, z_low, z_high)
    translate!(elements[:model], Vec3f(position.x, position.y, position.z))

    rotate!(elements[:model], to_makie_quaternion(orientation))
end

# let

#     q = to_makie_quaternion(QuatRotation(0, 0, 1, 0))

#     rotate!(elements[:model], q)
# end

