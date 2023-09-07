
function second_window_interactions(elements)

    fig_2 = gui_second_window_setup(folder_path * config_params_path)
    elements[:fig_2] = fig_2

    open_menu_btn = elements[:widgets][:open_menu_btn]

    on(open_menu_btn.clicks) do state

        Core.println("Menu btn clicked")

        # open second window
        screen2 = GLMakie.Screen()
        display(screen2, fig_2)
        GLMakie.set_screen_visibility!(screen2, true)

    end

end

