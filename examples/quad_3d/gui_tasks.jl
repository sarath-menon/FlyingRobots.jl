


function gui_dynamic_plotter(flag, c1, elements, df_empty)

    # df_empty = DataFrame()
    # delete all existing entries in the dataframe
    deleteat!(df_empty, :)

    # axis limits
    x_range = 20

    x_low = 0
    x_high = x_range

    y_max = ones(3) * 0.1
    y_max_padding = 0.1

    y_local_max = zeros(3)

    # set the initial axis limits
    FlyingRobots.Gui.set_2dplot_axislimits(plot_elements; x_low=x_low, x_high=x_high, y_max=y_max)

    state_plots = elements[:plots_2d][:state_plots]

    #Core.println("Waiting for sol data")
    while true

        # get the latest ODESolution subset from the channel
        sol = take!(c1)

        # convert the ODESolution to a dataframe
        df = sim_logging(sol)

        # append it to the main dataframr
        append!(df_empty, df)

        #compute dynamic axis limits 
        for i = 1:3

            # check the maximum value in the data to be plotted
            y_local_max[i] = maximum(df[!, "(quad1.rb.r(t), $i)"]) + y_max_padding

            # it's it's higher than the current y axis limits, increase the y axis limits
            if y_local_max[i] > y_max[i]
                y_max[i] = y_local_max[i]

                state_plots[i].limits = (x_low, x_high, -y_max[i], y_max[i])
            end
        end

        # check if it's time to change x axis limits
        if df[end, "timestamp"] > x_high

            # set new x_range
            x_low += x_range
            x_high += x_range

            FlyingRobots.Gui.set_2dplot_axislimits(plot_elements; x_low=x_low, x_high=x_high, y_max=y_max)

            # delete data from the prev x axis range since it's not being plotted anymore
            deleteat!(df_empty, :)
        end

        # do the actual plotting
        FlyingRobots.Gui.plot_position_dynamic(elements, df_empty)

        # ## @show sol.t[end]
        # Core.println(df[!, "timestamp"])

        if flag[] == false
            if isempty(c1)
                break
            end
        end

    end

    Core.println("Receiver task exiting")
end
