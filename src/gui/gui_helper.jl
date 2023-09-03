# # convert standrard quaternion, euler angle to makie quaternin type
function to_makie_quaternion(q::QuatRotation)
    return Quaternionf(q.q.v1, q.q.v2, q.q.v3, q.q.s)
end

function to_std_quaternion(q::Quaternion)
    return QuatRotation(q.q_w, q.q_x, q.q_y, q.q_z)
end

function rotate_mesh(m, q)
    makie_q = to_makie_quaternion(q)
    GLMakie.rotate!(m, makie_q)
end

function get_primary_resolution(index::Int)
    monitors = GLMakie.GLFW.GetMonitors()

    videomode = GLMakie.MonitorProperties(monitors[index]).videomode
    (xscale, yscale) = GLMakie.GLFW.GetMonitorContentScale(monitors[index])

    (height, width) = (videomode.width * xscale, videomode.height * yscale)
    (height, width) = (convert(Int, height), convert(Int, width))
    return (height, width)
end

function start_3d_animation(elements; duration=10.0, dt=0.01, frame_rate=25)

    df = elements[:df]

    sim_state = elements[:sim_state]
    sim_time = elements[:sim_time]

    timeline_slider = elements[:widgets][:timeline_slider]
    timeline_btn = elements[:widgets][:timeline_btn]

    step_count = floor(Int, duration / dt)
    n_skip_frames = floor(Int, 100 / frame_rate)

    @show sim_state

    # check if simulation is already running
    if sim_state[] == true
        return false
    end

    # set sim state flag to true
    sim_state[] = true

    # change button text to show "Stop"
    timeline_btn.label = "Stop"

    animation_loop = @async begin
        for i in 1:n_skip_frames:step_count

            # stop simulation is stop button is pressed
            if sim_state[] == false
                break
            end

            position = Vec3d(df[!, 2][i], df[!, 3][i], df[!, 4][i])
            orientation = QuatRotation(df[!, 8][i], df[!, 9][i], df[!, 10][i], df[!, 11][i])

            model_set_attitude_closeup_visualizer(elements, orientation)

            model_set_pose_fullscene_visualizer(elements, position, orientation)

            # set the time 
            sim_time[] = round(df[!, "timestamp"][i], digits=2)

            # set timeline slider value 
            set_close_to!(timeline_slider, sim_time[])

            # account for skipped frames in sleep time
            sleep(n_skip_frames * dt)

        end
    end

    @async begin
        # wait for animtion to finish
        wait(animation_loop)

        println("animation done")

        stop_3d_animation(elements)
    end
    return animation_loop

end

struct PlotData9
    time_vec::Observable{Vector{Float64}}

    state::Vector{Observable{Vector{Float64}}}
    reference::Vector{Observable{Vector{Float64}}}
    control::Vector{Observable{Vector{Float64}}}

    PlotData9(; time_vec, state, reference, control) = new(time_vec, state, reference, control)
end

PlotData = PlotData9

function model_set_pose_closeup_visualizer(elements, position, orientation)

    # plot_params = elements[:plot_params]
    vis_3d_params = elements[:params][:closeup_visualizer]

    max_range = vis_3d_params.axis.high - vis_3d_params.axis.low
    dist = max_range / 2

    x_low = position.x - dist
    y_low = position.y - dist
    z_low = position.z - dist

    x_high = position.x + dist
    y_high = position.y + dist
    z_high = position.z + dist

    closeup_model = elements[:closeup_visualizer][:model]

    # set pose in closeup visualizer 
    elements[:closeup_visualizer][:axis].limits = (x_low, x_high, y_low, y_high, z_low, z_high)

    translate!(closeup_model, Vec3f(position.x, position.y, position.z))
    rotate!(closeup_model, to_makie_quaternion(orientation))
end

function model_set_attitude_closeup_visualizer(elements, orientation)

    # plot_params = elements[:plot_params]
    vis_3d_params = elements[:params][:closeup_visualizer]

    closeup_model = elements[:closeup_visualizer][:model]

    rotate!(closeup_model, to_makie_quaternion(orientation))
end

function model_set_pose_fullscene_visualizer(elements, position, orientation)

    fullscene_model = elements[:fullscene_visualizer][:model]

    # set pose in fullscale visualizer 
    translate!(fullscene_model, Vec3f(position.x, position.y, position.z))
    rotate!(fullscene_model, to_makie_quaternion(orientation))
end



