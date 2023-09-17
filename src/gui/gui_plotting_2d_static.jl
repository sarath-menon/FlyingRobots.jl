
function plot_position(elements)

    df = elements[:df]
    plots = elements[:plots_2d][:state_plots]
    plots_2d_data = elements[:plots_2d_data]

    plots_2d_data.time_vec[] = df[!, "timestamp"]

    plots_2d_data.state[1][] = df[!, "(quad1.rb.r(t), 1)"]
    plots_2d_data.state[2][] = df[!, "(quad1.rb.r(t), 2)"]
    plots_2d_data.state[3][] = df[!, "(quad1.rb.r(t), 3)"]

    plots_2d_data.reference[1][] = df[!, "(controller.r_ref(t), 1)"]
    plots_2d_data.reference[2][] = df[!, "(controller.r_ref(t), 2)"]
    plots_2d_data.reference[3][] = df[!, "(controller.r_ref(t), 3)"]

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


function plot_orientation(elements)

    df = elements[:df]
    plots = elements[:plots_2d][:state_plots]
    plots_2d_data = elements[:plots_2d_data]

    plots_2d_data.time_vec[] = df[!, "timestamp"]

    qs = df[!, "(quad1.rb.q(t), 1)"]
    q1 = df[!, "(quad1.rb.q(t), 2)"]
    q2 = df[!, "(quad1.rb.q(t), 3)"]
    q3 = df[!, "(quad1.rb.q(t), 4)"]

    qs_ref = df[!, "(controller.q_ref(t), 1)"]
    q1_ref = df[!, "(controller.q_ref(t), 2)"]
    q2_ref = df[!, "(controller.q_ref(t), 3)"]
    q3_ref = df[!, "(controller.q_ref(t), 4)"]

    quat_vec = QuatRotation.(qs, q1, q2, q3, false)
    quat_vec_ref = QuatRotation.(qs_ref, q1_ref, q2_ref, q3_ref, false)

    # convert quaternion attitude representation to euler angles
    res_vec = Rotations.params.(RotZYX.(quat_vec))
    res_vec_ref = Rotations.params.(RotZYX.(quat_vec_ref))

    res_matrix = reduce(hcat, res_vec)
    res_matrix_ref = reduce(hcat, res_vec_ref)

    r_vec = res_matrix[1, :]
    q_vec = res_matrix[2, :]
    p_vec = res_matrix[3, :]

    r_vec_ref = res_matrix_ref[1, :]
    q_vec_ref = res_matrix_ref[2, :]
    p_vec_ref = res_matrix_ref[3, :]

    plots_2d_data.state[1][] = rad2deg.(p_vec)
    plots_2d_data.state[2][] = rad2deg.(q_vec)
    plots_2d_data.state[3][] = rad2deg.(r_vec)

    plots_2d_data.reference[1][] = rad2deg.(p_vec_ref)
    plots_2d_data.reference[2][] = rad2deg.(q_vec_ref)
    plots_2d_data.reference[3][] = rad2deg.(r_vec_ref)

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


function plot_angular_velocity(elements)

    df = elements[:df]
    plots = elements[:plots_2d][:state_plots]
    plots_2d_data = elements[:plots_2d_data]

    plots_2d_data.time_vec[] = df[!, "timestamp"]

    ω_x_rad = df[!, "(quad1.rb.ω(t), 1)"]
    ω_y_rad = df[!, "(quad1.rb.ω(t), 2)"]
    ω_z_rad = df[!, "(quad1.rb.ω(t), 3)"]

    plots_2d_data.state[1][] = rad2deg.(ω_x_rad)
    plots_2d_data.state[2][] = rad2deg.(ω_y_rad)
    plots_2d_data.state[3][] = rad2deg.(ω_z_rad)

    plots_2d_data.reference[1][] = df[!, "(controller.ω_ref(t), 1)"]
    plots_2d_data.reference[2][] = df[!, "(controller.ω_ref(t), 2)"]
    plots_2d_data.reference[3][] = df[!, "(controller.ω_ref(t), 3)"]

    autolimits!(plots[1])
    autolimits!(plots[2])
    autolimits!(plots[3])

    # set axis titles
    plots[1].title = "x angular velocity"
    plots[2].title = "y angular velocity"
    plots[3].title = "z angular velocity"

    # set axis labels
    plots[1].ylabel = "angular velocity [deg/s]"
    plots[2].ylabel = "angular velocity [deg/s]"
    plots[3].ylabel = "angular velocity [deg/s]"
end

function plot_control_input(elements, motor_thrust_to_body_thrust)

    df = elements[:df]
    plots = elements[:plots_2d][:control_plots]
    plots_2d_data = elements[:plots_2d_data]

    plots_2d_data.time_vec[] = df[!, "timestamp"]
    n_timesteps = size(df)[1]

    # convert motor thrusts to body thrust, torque
    f_net_vec = Float64[]
    τ_x_vec = Float64[]
    τ_y_vec = Float64[]
    τ_z_vec = Float64[]

    # find index of motor_1 thrust 
    # id = findfirst(x -> x == "(controller.U(t), 1)", names(df))

    M = motor_thrust_to_body_thrust(l=vehicle_params.arm_length, k_τ=vehicle_params.actuators.constants.k_τ)

    motor_thrusts = zeros(4, n_timesteps)

    motor_thrusts[1, :] = df[!, "(controller.U(t), 1)"]
    motor_thrusts[2, :] = df[!, "(controller.U(t), 2)"]
    motor_thrusts[3, :] = df[!, "(controller.U(t), 3)"]
    motor_thrusts[4, :] = df[!, "(controller.U(t), 4)"]

    result = M * motor_thrusts

    f_net = result[1, :]
    τ_x = result[2, :]

    plots_2d_data.control[1][] = f_net
    plots_2d_data.control[2][] = τ_x
    # plots_2d_data.axis_3[] = r_vec

    # autolimits!(plots[1])
    # autolimits!(plots[2])
    # autolimits!(plots[3])

    # set axis titles
    plots[1].title = "body thrust"
    plots[2].title = "x axis torque"

    # set axis labels
    plots[1].ylabel = "thrust [N]"
    plots[2].ylabel = "pitch angle [°]"
    # plots[3].ylabel = "yaw angle [°]"
end