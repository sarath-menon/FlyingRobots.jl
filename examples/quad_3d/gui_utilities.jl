
function plot_position(state_plots, plot_data, sol)
    plot_data.time_vec[] = sol.t

    plot_data.axis_1[] = sol[plant.r[1]]
    plot_data.axis_2[] = sol[plant.r[2]]
    plot_data.axis_3[] = sol[plant.r[3]]

    # set axis limits 
    # xlims!(Gui.state_plots[1], 0, sol.t[end])
    # xlims!(Gui.state_plots[2], 0, sol.t[end])
    # xlims!(Gui.state_plots[3], 0, sol.t[end])

    autolimits!(Gui.state_plots[1])
    autolimits!(Gui.state_plots[2])
    autolimits!(Gui.state_plots[3])

    # set axis titles
    state_plots[1].title = "x position"
    state_plots[2].title = "y position"
    state_plots[3].title = "z position"

    # set axis labels
    state_plots[1].ylabel = "x pos [m]"
    state_plots[2].ylabel = "y pos [m]"
    state_plots[3].ylabel = "z pos [m]"
end


