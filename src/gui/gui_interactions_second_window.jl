
function define_interactions_second_window(elements)
    fig_2 = elements[:second_window][:fig]

    open_menu_btn = elements[:widgets][:open_menu_btn]

    on(open_menu_btn.clicks) do state
        Core.println("Menu btn clicked")

        screen2 = elements[:second_window][:screen]

        if elements[:second_window][:shown] == true

            GLMakie.set_screen_visibility!(screen2, false)

            elements[:second_window][:shown] = false
        else
            GLMakie.set_screen_visibility!(screen2, true)

            elements[:second_window][:shown] = true
        end
    end
end

