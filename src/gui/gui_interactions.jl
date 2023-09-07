
function define_interactions(elements, sim_time)

    time_title = elements[:titles][:time_title]
    anim_state = elements[:anim_state]
    sim_cmd = elements[:sim_cmd]
    sim_acc_mode = elements[:sim_acc_mode]

    # to set time in title
    on(sim_time) do time
        time_title.text[] = "Time: " * string(time) * " s"
    end

    # change displayed time according to slider position
    timeline_slider = elements[:widgets][:timeline_slider]
    play_btn = elements[:widgets][:play_btn]
    start_sim_btn = elements[:widgets][:start_sim_btn]

    lift(timeline_slider.value) do val
        sim_time[] = val
    end

    # play 3d visualization if 'Play' button is clicked
    on(play_btn.clicks) do clicks

        # if sim is not already running, start sim
        if anim_state[] == false

            start_3d_animation(elements)
        else
            stop_3d_animation(elements)
        end
    end

    # start stepping sim if start_sim_btn is clicked
    on(start_sim_btn.clicks) do clicks

        # if sim is not already running, start sim
        if sim_cmd[] == SimIdle()

            # start_3d_animation(elements)
            sim_cmd[] = SimRunning()

            Core.println("Start sim button pressed")

            # stop func reqd only for realtime sim
            if sim_acc_mode[] == RealtimeSim()
                start_sim_btn.label = "Stop Sim"
            end

            # if sim is running, stop sim
        elseif sim_cmd[] == SimRunning()

            # in relatime mode, press during sim running means stop.

            if sim_acc_mode[] == RealtimeSim()
                sim_cmd[] = SimIdle()

                start_sim_btn.label = "Start Sim"
                Core.println("Stop sim button pressed")

                # In accelerated mode, it means start nex sim
            elseif sim_acc_mode[] == AcceleratedSim()

                sim_cmd[] = SimRunning()

                Core.println("Starting next accelerated sim")
            end
        end


    end

    sim_acc_toggle = elements[:widgets][:sim_acc_toggle]
    sim_acc_toggle_label = elements[:widgets][:sim_acc_toggle_label]

    on(sim_acc_toggle.active) do state

        # stop current sim  and set button to "Start" if sim mode is changed 
        start_sim_btn.label = "Start Sim"
        sim_cmd[] = SimIdle()

        if state == true
            Core.println("Sim mode set to Accelerated")
            sim_acc_toggle_label.text = "Accelerated Sim"

            sim_acc_mode[] = AcceleratedSim()

        else
            Core.println("Sim mode set to Realtime")
            sim_acc_toggle_label.text = "Realtime Sim"

            sim_acc_mode[] = RealtimeSim()
        end
    end
end


