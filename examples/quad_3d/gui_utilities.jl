
function plot_position(plots, plot_data, sol, state_indices)

    i = state_indices.position[1]

    plot_data.time_vec[] = sol.t

    plot_data.axis_1[] = sol[i, :]
    plot_data.axis_2[] = sol[i+1, :]
    plot_data.axis_3[] = sol[i+2, :]

    autolimits!(plots[1])
    autolimits!(plots[2])
    autolimits!(plots[3])

    # set axis titles
    plots[1].title = "x position"
    plots[2].title = "y position"
    plots[3].title = "z position"

    # set axis labels
    plots[1].ylabel = "pos [m]"
    plots[2].ylabel = "pos [m]"
    plots[3].ylabel = "pos [m]"
end


function plot_orientation(plots, plot_data, sol, state_indices)

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

    autolimits!(plots[1])
    autolimits!(plots[2])
    autolimits!(plots[3])

    # set axis titles
    plots[1].title = "roll angle"
    plots[2].title = "pitch angle"
    plots[3].title = "yaw angle"

    # set axis labels
    plots[1].ylabel = "angle [°]"
    plots[2].ylabel = "angle [°]"
    plots[3].ylabel = "angle [°]"
end


function plot_control_input(plots, plot_data, sol, state_indices)

    id = state_indices.motor_thrusts[1]
    plot_data.time_vec[] = sol.t

    # convert motor thrusts to body thrust, torque
    f_net_vec = Float64[]
    τ_x_vec = Float64[]
    τ_y_vec = Float64[]
    τ_z_vec = Float64[]

    M = motor_thrust_to_body_thrust(l=0.7, k_τ=0.0035)

    for i in 1:length(sol.t)

        motor_thrusts = sol.u[i][id:id+3]
        result = M * motor_thrusts

        f_net = result[1]
        τ = result[2:4]

        push!(f_net_vec, f_net)
        push!(τ_x_vec, τ[1])
        # push!(τ_y_vec, τ[2])
        # push!(τ_z_vec, τ[3])
    end

    plot_data.axis_4[] = f_net_vec
    plot_data.axis_5[] = τ_x_vec
    # plot_data.axis_3[] = r_vec

    autolimits!(plots[1])
    autolimits!(plots[2])
    # autolimits!(plots[3])

    # set axis titles
    plots[1].title = "body thrust"
    plots[2].title = "x axis torque"

    # set axis labels
    plots[1].ylabel = "thrust [N]"
    plots[2].ylabel = "pitch angle [°]"
    # plots[3].ylabel = "yaw angle [°]"
end

