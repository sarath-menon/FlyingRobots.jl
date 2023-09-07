
function define_interactions_second_window(elements)
    fig_2 = elements[:second_window][:fig]

    open_menu_btn = elements[:widgets][:open_menu_btn]

    on(open_menu_btn.clicks) do state
        Core.println("Menu btn clicked")

        # if elements[:second_window][:shown] == false

        # open second window
        screen2 = GLMakie.Screen()
        display(screen2, fig_2)

        elements[:second_window][:screen] = screen2

        elements[:second_window][:shown] = true
        # end
    end
end

