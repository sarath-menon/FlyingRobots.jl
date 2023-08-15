export quad_2d_plot_normal, quad_2d_plot_lsim, quad2d_plot_initialize

using GLMakie
using DifferentialEquations

struct Quad2dPlot4
    fig::Figure

    y_ax::Axis
    z_ax::Axis
    theta_ax::Axis

    thrust_ax::Axis
    torque_ax::Axis

    time_vec::Observable{Vector{Float64}}

    y_vec::Observable{Vector{Float64}}
    z_vec::Observable{Vector{Float64}}
    theta_vec::Observable{Vector{Float64}}

    thrust_vec::Observable{Vector{Float64}}
    torque_vec::Observable{Vector{Float64}}
end

Quad2dPlot = Quad2dPlot4

#------------------------------------------------------------------

function quad2d_plot_initialize(frmodel_params::FrModel, tspan::Tuple)
    fig = Figure(resolution=(1200, 600))
    dt = frmodel_params.Ts

    range = tspan[1]:dt:(tspan[2])
    len::Integer = length(range)

    # state variables 
    y_ax = Axis(fig[1, 1], title="y")
    z_ax = Axis(fig[2, 1], title="z")
    theta_ax = Axis(fig[3, 1], title="theta")

    # control input 
    thrust_ax = Axis(fig[1, 2], title="Thrust")
    torque_ax = Axis(fig[2, 2], title="Torque")

    y_ax.ylabel = "y [m]"
    z_ax.ylabel = "z [m]"
    theta_ax.ylabel = "θ [rad]"

    thrust_ax.ylabel = "Thrust [N]"
    torque_ax.ylabel = "Toruqe [Nm]"

    rowgap!(fig.layout, 1)
    display(fig)

    # Intial plot data 
    y_vec = Observable{Vector{Float64}}(zeros(len))
    z_vec = Observable{Vector{Float64}}(zeros(len))
    theta_vec = Observable{Vector{Float64}}(zeros(len))

    thrust_vec = Observable{Vector{Float64}}(zeros(len))
    torque_vec = Observable{Vector{Float64}}(zeros(len))

    time_vec = Observable{Vector{Float64}}(collect(range))

    # Plot initial data (zeros)
    lines!(y_ax, time_vec, y_vec)
    # lines!(plot.z_ax, time_vec, z_ref, linestyle=:dash)

    lines!(z_ax, time_vec, z_vec)
    # lines!(plot.y_ax, time_vec, y_ref, linestyle=:dash)

    lines!(theta_ax, time_vec, theta_vec)
    # lines!(plot.theta_ax, time_vec, theta_ref, linestyle=:dash)

    lines!(thrust_ax, time_vec, thrust_vec)
    lines!(torque_ax, time_vec, torque_vec)

    return Quad2dPlot(fig, y_ax, z_ax, theta_ax, thrust_ax, torque_ax,
        time_vec, y_vec, z_vec, theta_vec, thrust_vec, torque_vec)
end


function quad_2d_plot_normal(plot::Quad2dPlot, sol::ODESolution; y_ref, z_ref, theta_ref)
    f_1 = @view sol[7, :]
    f_2 = @view sol[8, :]

    thrust = f_1 + f_2
    torque = (f_1 - f_2) * 0.1

    # compute axis limits
    t_axis_low = sol.t[1]
    t_axis_high = sol.t[end]

    (y_low, y_high) = extrema(sol[1, :])
    (z_low, z_high) = extrema(sol[2, :])
    (theta_low, theta_high) = extrema(sol[3, :])

    # set axis limits 
    plot.y_ax.limits = (t_axis_low, t_axis_high, y_low, y_high)
    plot.z_ax.limits = (t_axis_low, t_axis_high, z_low, z_high)
    plot.theta_ax.limits = (t_axis_low, t_axis_high, theta_low, theta_high)

    # set axis data
    plot.time_vec[] = sol.t

    plot.y_vec[] = sol[1, :]
    plot.z_vec[] = sol[2, :]
    plot.theta_vec[] = sol[3, :]

    # plot.thrust_vec[] = thrust
    # plot.torque_vec[] = torque
end

function quad_2d_plot_normal(plot::Quad2dPlot, logger::Logger; y_ref, z_ref, theta_ref)
    # f_1 = @view sol[7, :]
    # f_2 = @view sol[8, :]

    # thrust = f_1 + f_2
    # torque = (f_1 - f_2) * 0.1

    log_matrix = logger.log_matrix

    # compute axis limits
    t_axis_low = log_matrix[1, 1]
    t_axis_high = log_matrix[6000, 1]

    time_vec = @view log_matrix[:, 1]
    y_vec = @view log_matrix[:, 2]
    z_vec = @view log_matrix[:, 3]
    θ_vec = @view log_matrix[:, 4]

    (y_low, y_high) = extrema(y_vec)
    (z_low, z_high) = extrema(z_vec)
    (theta_low, theta_high) = extrema(θ_vec)

    # set axis limits 
    plot.y_ax.limits = (t_axis_low, t_axis_high, y_low, y_high)
    plot.z_ax.limits = (t_axis_low, t_axis_high, z_low, z_high)
    plot.theta_ax.limits = (t_axis_low, t_axis_high, theta_low, theta_high)

    # set axis data
    plot.time_vec[] = time_vec

    plot.y_vec[] = y_vec
    lines!(plot.y_ax, time_vec, y_ref, linestyle=:dash)

    plot.z_vec[] = z_vec
    lines!(plot.z_ax, time_vec, z_ref, linestyle=:dash)

    plot.theta_vec[] = θ_vec
    lines!(plot.theta_ax, time_vec, theta_ref, linestyle=:dash)


    # plot.thrust_vec[] = thrust
    # plot.torque_vec[] = torque
end

