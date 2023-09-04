
function set_2dplot_axislimits(elements; x_low, x_high, y_max)

    state_plots = elements[:plots_2d][:state_plots]
    control_plots = elements[:plots_2d][:state_plots]

    for i = 1:3
        state_plots[i].limits = (x_low, x_high, -y_max[i], y_max[i])
    end
end

function plot_position_dynamic(elements, df)

    plots = elements[:plots_2d][:state_plots]
    plots_2d_data = elements[:plots_2d_data]

    plots_2d_data.time_vec[] = df[!, "timestamp"]

    # 2D plotting
    for i = 1:3
        # set state data 
        plots_2d_data.state[i][] = df[!, "(quad1.rb.r(t), $i)"]

        # set reference 
        plots_2d_data.reference[i][] = df[!, "(controller.R(t), $i)"]
    end

    # 3d animation
    if nrow(df) != 0
        position = Vec3d(df[!, 2][end], df[!, 3][end], df[!, 4][end])
        orientation = QuatRotation(df[!, 8][end], df[!, 9][end], df[!, 10][end], df[!, 11][end])

        model_set_attitude_closeup_visualizer(elements, orientation)

        model_set_pose_fullscene_visualizer(elements, position, orientation)

    end
end

