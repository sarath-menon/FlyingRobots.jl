




function gui_receiver(elements, c1, lock_handle)

    if trylock(lock_handle) == false
        println("Already locked")
        return
    end

    try
        sim_cmd = elements[:sim_cmd]

        # circular c_buffer
        c_buffer_len = 10
        c_buffer = CircularDeque{ODESolution}(c_buffer_len)

        elements[:receiver_buffer] = c_buffer

        Core.println("Waiting for sim data")

        while true
            # take data from the channel
            sol = take!(c1)

            if length(c_buffer) == c_buffer_len
                pop!(c_buffer)
            end

            pushfirst!(c_buffer, sol)

            if sim_cmd[] == SimIdle()
                if isempty(c1)
                    break
                end
            end
        end

    catch
        println("Exception: Killing gui receiver")

    finally
        delete!(elements, :receiver_buffer)
        unlock(lock_handle)
        Core.println("Gui receiver terminated")
    end
end


function gui_dynamic_plotter(elements, df_empty, lock_handle)

    if trylock(lock_handle) == false
        println("Already locked")
        return
    end

    try
        sim_cmd = elements[:sim_cmd]

        deleteat!(df_empty, :)

        # axis limits
        x_range = 20

        x_low = 0
        x_high = x_range

        y_max = ones(3) * 0.1
        y_max_padding = 0.1

        y_local_max = zeros(3)

        # set the initial axis limits
        set_2dplot_axislimits(elements; x_low=x_low, x_high=x_high, y_max=y_max)

        state_plots = elements[:plots_2d][:state_plots]

        if haskey(elements, :receiver_buffer)
            c_buffer = elements[:receiver_buffer]
        else
            throw("Gui receiver must be running to used 3d plotter")
        end

        while true

            while length(c_buffer) == 0
                sleep(0.01)
            end

            sol = pop!(c_buffer)

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

                set_2dplot_axislimits(elements; x_low=x_low, x_high=x_high, y_max=y_max)

                # delete data from the prev x axis range since it's not being plotted anymore
                deleteat!(df_empty, :)
            end

            # do the actual plotting
            plot_position_dynamic(elements, df_empty)

            # Core.println(df[!, "timestamp"])

            if sim_cmd[] == SimIdle()
                break
            end
        end

    catch e
        println("Exception: Killing gui 3d plotter")

    finally
        unlock(lock_handle)

        Core.println("Gui 3d plotter terminated")
    end
end



function wait_until(c::Condition; timeout::Real)
    timer = Timer(timeout) do t

        Core.println("Shutting down GUI plotter - no data received from sim ")
        return 0
    end

    return wait(c)
end