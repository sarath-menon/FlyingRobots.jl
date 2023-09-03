
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

    for i = 1:3
        # set state data 
        plots_2d_data.state[i][] = df[!, "(quad1.rb.r(t), $i)"]

        # set reference 
        plots_2d_data.reference[i][] = df[!, "(controller.R(t), $i)"]

    end
end

