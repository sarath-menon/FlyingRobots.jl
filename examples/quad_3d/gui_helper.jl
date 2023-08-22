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

function plot_3d_trajectory(; sim_time_obs::Observable, sim_state_obs::Observable, duration=10.0, dt=0.01, frame_rate=25)

    step_count::Integer = convert(Integer, duration / dt)

    n_skip_frames::Int8 = convert(Int8, 100 / frame_rate)

    for i in 1:n_skip_frames:step_count

        # stop simulation is stop button is pressed
        if sim_state_obs[] == true

            max_range = plot_params.visualizer.axis.high - plot_params.visualizer.axis.low
            dist = max_range / 2

            x_pos::Float64 = 0.0
            y_pos::Float64 = df[!, "y"][i]
            z_pos::Float64 = df[!, "z"][i]

            x_low::Float64 = x_pos - dist
            y_low::Float64 = y_pos - dist
            z_low::Float64 = z_pos - dist

            x_high::Float64 = x_pos + dist
            y_high::Float64 = y_pos + dist
            z_high::Float64 = z_pos + dist

            # set the time observable
            sim_time::Float64 = round(df[!, "timestamp"][i], digits=2)
            sim_time_obs[] = sim_time

            # set timeline slider value 
            set_close_to!(timeline_slider, sim_time)

            vis_ax.limits = (x_low, x_high, y_low, y_high, z_low, z_high)
            translate!(m, Vec3f(x_pos, y_pos, z_pos))

            rotate!(m, Vec3f(1, 0, 0), rad2deg(df[!, "θ"][i]))

            sleep(n_skip_frames * dt)

        end
    end


end