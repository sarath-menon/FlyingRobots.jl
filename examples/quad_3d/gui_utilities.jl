
function plot_position(plots, plot_data, df::DataFrame)

    plot_data.time_vec[] = df[!, "timestamp"]

    plot_data.axis_1[] = df[!, "(quad1.rb.r(t), 1)"]
    plot_data.axis_2[] = df[!, "(quad1.rb.r(t), 2)"]
    plot_data.axis_3[] = df[!, "(quad1.rb.r(t), 3)"]

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

function plot_orientation(plots, plot_data, df)

    plot_data.time_vec[] = df[!, "timestamp"]

    qs = log_sim_output[!, "(quad1.rb.q(t), 1)"]
    q1 = log_sim_output[!, "(quad1.rb.q(t), 2)"]
    q2 = log_sim_output[!, "(quad1.rb.q(t), 3)"]
    q3 = log_sim_output[!, "(quad1.rb.q(t), 4)"]

    quat_vec = QuatRotation.(qs, q1, q2, q3, false)

    # convert quaternion attitude representation to euler angles
    res_vec = Rotations.params.(RotZYX.(quat_vec))

    res_matrix = reduce(hcat, res_vec)

    r_vec = res_matrix[1, :]
    q_vec = res_matrix[2, :]
    p_vec = res_matrix[3, :]

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

function plot_control_input(plots, plot_data, df, vehicle_params)

    plot_data.time_vec[] = df[!, "timestamp"]
    n_timesteps = size(log_sim_output)[1]

    # convert motor thrusts to body thrust, torque
    f_net_vec = Float64[]
    τ_x_vec = Float64[]
    τ_y_vec = Float64[]
    τ_z_vec = Float64[]

    # find index of motor_1 thrust 
    # id = findfirst(x -> x == "(controller.U(t), 1)", names(log_sim_output))

    M = motor_thrust_to_body_thrust(l=vehicle_params.arm_length, k_τ=vehicle_params.actuators.constants.k_τ)

    motor_thrusts = zeros(4, n_timesteps)

    motor_thrusts[1, :] = log_sim_output[!, "(controller.U(t), 1)"]
    motor_thrusts[2, :] = log_sim_output[!, "(controller.U(t), 2)"]
    motor_thrusts[3, :] = log_sim_output[!, "(controller.U(t), 3)"]
    motor_thrusts[4, :] = log_sim_output[!, "(controller.U(t), 4)"]

    result = M * motor_thrusts

    f_net = result[1, :]
    τ_x = result[2, :]

    plot_data.axis_4[] = f_net
    plot_data.axis_5[] = τ_x
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