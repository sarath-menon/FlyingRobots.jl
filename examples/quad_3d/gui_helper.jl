

function define_gui_sim_interactions(plot_elements, sim_gui_ch, df_empty)

    sim_cmd = plot_elements[:sim_cmd]
    sim_acc_mode = plot_elements[:sim_acc_mode]

    obs_func = on(sim_cmd, weak=true) do val
        if sim_cmd[] == SimRunning()

            if sim_acc_mode[] == RealtimeSim()
                Core.println("Starting Realtime simulation")
                start_realtime_sim(plot_elements, sim_gui_ch, df_empty)

            elseif sim_acc_mode[] == AcceleratedSim()
                Core.println("Starting Accelerated simulation")

                start_accelerated_sim(plot_elements, sim_gui_ch)
            end
        end
    end

    return obs_func
end

function start_realtime_sim(plot_elements, sim_gui_ch, df_empty)

    sim_cmd = plot_elements[:sim_cmd]
    sim_acc_mode = plot_elements[:sim_acc_mode]

    sim_gui_channel_lock = plot_elements[:locks][:sim_channel]
    gui_recv_buffer_lock = plot_elements[:locks][:recv_buffer]

    # clear existing plot data 
    FlyingRobots.Gui.plot_reset(plot_elements)

    @async FlyingRobots.Gui.gui_receiver(plot_elements, sim_gui_ch, sim_gui_channel_lock)
    @async FlyingRobots.Gui.gui_dynamic_plotter(plot_elements, df_empty, gui_recv_buffer_lock)

    sim_task = @tspawnat 2 run_sim_stepping(sys, subsystems, sim_gui_ch, sim_cmd, sim_acc_mode; save=false)

    return sim_task
end

function start_accelerated_sim(plot_elements, sim_gui_ch)

    sim_cmd = plot_elements[:sim_cmd]
    sim_acc_mode = plot_elements[:sim_acc_mode]

    # clear existing plot data 
    FlyingRobots.Gui.plot_reset(plot_elements)

    # run sim on 2nd thread
    @time sim_task = @tspawnat 2 run_sim_stepping(sys, subsystems, sim_gui_ch, sim_cmd, sim_acc_mode; save=false)
    df = fetch(sim_task)

    # plot the result
    FlyingRobots.Gui.set_sim_instance(plot_elements, df)
    FlyingRobots.Gui.plot_position(plot_elements)
end
