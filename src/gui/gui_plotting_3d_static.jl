function start_3d_animation(elements; duration=10.0, dt=0.01, frame_rate=25)

    df = elements[:df]

    anim_state = elements[:anim_state]
    sim_time = elements[:sim_time]

    timeline_slider = elements[:widgets][:timeline_slider]
    play_btn = elements[:widgets][:play_btn]

    step_count = floor(Int, duration / dt)
    n_skip_frames = floor(Int, 100 / frame_rate)

    @show anim_state

    # check if simulation is already running
    if anim_state[] == true
        return false
    end

    # set sim state flag to true
    anim_state[] = true

    # change button text to show "Stop"
    play_btn.label = "Stop"

    animation_loop = @async begin
        for i in 1:n_skip_frames:step_count

            # stop simulation is stop button is pressed
            if anim_state[] == false
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




function stop_3d_animation(elements)
    anim_state = elements[:anim_state]
    play_btn = elements[:widgets][:play_btn]

    # if sim is currently running, set sim state flag to false
    anim_state[] = false

    # change button text to show "Stop"
    play_btn.label = "Play"
end
