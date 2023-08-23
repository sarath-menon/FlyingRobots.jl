
function plot_position(state_plots, plot_data, sol, state_indices)

    i = state_indices.position[1]

    plot_data.time_vec[] = sol.t

    plot_data.axis_1[] = sol[i, :]
    plot_data.axis_2[] = sol[i+1, :]
    plot_data.axis_3[] = sol[i+2, :]

    autolimits!(state_plots[1])
    autolimits!(state_plots[2])
    autolimits!(state_plots[3])

    # set axis titles
    state_plots[1].title = "x position"
    state_plots[2].title = "y position"
    state_plots[3].title = "z position"

    # set axis labels
    state_plots[1].ylabel = "x pos [m]"
    state_plots[2].ylabel = "y pos [m]"
    state_plots[3].ylabel = "z pos [m]"
end


functip

function plot_orientation(state_plots, plot_data, sol, state_indices)

    plot_data.time_vec[] = sol.t

    # convert quaternion attitude representation to euler angles 
    p_vec = Float64[]
    q_vec = Float64[]
    r_vec = Float64[]

    for i in 1:length(sol.t)
        quat_vec = QuatRotation(sol.u[i][7:10], false)
        r, q, p = Rotations.params(RotZYX(quat_vec))

        push!(p_vec, rad2deg(p))
        push!(q_vec, rad2deg(q))
        push!(r_vec, rad2deg(r))
    end

    plot_data.axis_1[] = p_vec
    plot_data.axis_2[] = q_vec
    plot_data.axis_3[] = r_vec

    autolimits!(state_plots[1])
    autolimits!(state_plots[2])
    autolimits!(state_plots[3])

    # set axis titles
    state_plots[1].title = "roll angle"
    state_plots[2].title = "pitch angle"
    state_plots[3].title = "yaw angle"

    # set axis labels
    state_plots[1].ylabel = "roll angle [°]"
    state_plots[2].ylabel = "pitch angle [°]"
    state_plots[3].ylabel = "yaw angle [°]"
end






