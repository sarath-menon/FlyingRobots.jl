


function gui_second_window_setup(elements, path)

    config_yaml = YAML.load_file(path; dicttype=Dict{Symbol,Any})
    config_params = recursive_dict_to_namedtuple(config_yaml)

    grids_vec = GridLayout[]
    tab_names_vec = String[]

    set_theme!(backgroundcolor=:white, textcolor=:black, fontsize=25)
    fig = Figure(resolution=(1920, 1080))

    for (i, tab) in enumerate(config_params.tabs)
        @show tab.name

        # menu grid
        config_grid = GridLayout(valign=:top, halign=:left, alignmode=Outside(40))
        push!(grids_vec, config_grid)

        # tab names
        push!(tab_names_vec, tab.name)
    end

    # left and right grids
    g_left = fig[1, 1] = GridLayout(valign=:top)
    # g_right = fig[1, 2] = GridLayout(valign=:top, halign=:left, alignmode=Outside(40))

    # hidden layouts
    hiddenlayout = GridLayout(bbox=BBox(-1000, -1000, 0, 0))

    # set left and right grids
    tab_id = 1
    # g_right = grids_vec[tab_id]

    # create left menu  --------------------------------------------------------
    g_left_menu = g_left[1, 1] = GridLayout(halign=:left, valign=:top, alignmode=Outside(40))
    buttongrid = g_left_menu[1, 1] = GridLayout(halign=:left, valign=:top)

    buttonlabels = tab_names_vec
    buttons = buttongrid[1:length(buttonlabels), 1] = [Button(fig, label=l, halign=:left, width=250, height=100, fontsize=30) for l in buttonlabels]

    # Add tab items --------------------------------------------------------

    for (i, tab) in enumerate(config_params.tabs)

        # widget counter
        id = 1

        # Dropdown menu
        if haskey(tab, :Menu)
            for menu in tab.Menu

                menu_handle = Menu(fig, options=menu.options, tellwidth=false)
                label = Label(fig, menu.name * ": ")
                grids_vec[i][id, 1] = hgrid!(label, menu_handle, tellwidth=false)

                id += 1
            end

        end

        # Sliders
        if haskey(tab, :Slider)
            for slider in tab.Slider

                slider_handle = Slider(fig, range=slider.min:0.01:slider.max,
                    startvalue=slider.startvalue, tellwidth=false, halign=:left)
                label = Label(fig, slider.name * ": ")
                grids_vec[i][id, 1] = hgrid!(label, slider_handle, tellwidth=false)

                id += 1
            end
        end

        # Text boxes
        if haskey(tab, :TextBox)
            for textbox in tab.TextBox

                textbox_handle = Textbox(fig, placeholder=textbox.placeholder,
                    validator=Float64, tellwidth=false, halign=:left)
                label = Label(fig, textbox.name * ": ")
                grids_vec[i][id, 1] = hgrid!(label, textbox_handle, tellwidth=false)

                id += 1
            end
        end

        # Buttons
        if haskey(tab, :Button)
            for button in tab.Button

                button_handle = Button(fig, label=button.label, tellwidth=false, width=170, height=70, halign=:left, fontsize=25)
                grids_vec[i][id, 1] = button_handle

                id += 1
            end
        end

    end


    # Interactions --------------------------------------------------------

    # initilaization
    grid_shown = 1
    fig[1, 2] = grids_vec[grid_shown]
    hiddenlayout[1, 1] = grids_vec[2]
    hiddenlayout[1, 1] = grids_vec[3]

    # # Add box
    # fig[1, 3] = Box(fig, color=:red)

    for i in 1:length(buttonlabels)
        on(buttons[i].clicks) do n

            # @show "pressed", i
            # move currently shown grid to hiddenlayput
            hiddenlayout[1, 1] = grids_vec[grid_shown]

            # add new grid to figure
            fig[1, 2] = grids_vec[i]

            grid_shown = i

        end
    end

    # display window --------------------------------------------------------
    # screen2 = GLMakie.Screen()
    # display(screen2, fig)

    # GLMakie.set_screen_visibility!(screen2, true)

    elements[:second_window] = Dict()

    elements[:second_window][:shown] = false

    elements[:second_window][:fig] = fig
end