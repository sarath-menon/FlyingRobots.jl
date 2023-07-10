export quad_2d_plot_normal, quad_2d_plot_lsim

struct Quad2dPlot2
    fig::Figure

    y_ax::Axis
    z_ax::Axis
    theta_ax::Axis

    thrust_ax::Axis
    torque_ax::Axis

end

Quad2dPlot = Quad2dPlot2
##

function quad2d_plot_initialize()
    fig = Figure(resolution=(1200, 600))

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

    return Quad2dPlot(fig, y_ax, z_ax, theta_ax, thrust_ax, torque_ax)
end


function quad_2d_plot_normal(plot::Quad2dPlot, sol::ODESolution; y_ref, z_ref, theta_ref)

    lines!(plot.y_ax, sol.t, sol[1, :])
    lines!(plot.z_ax, sol.t, z_ref, linestyle=:dash)

    lines!(plot.z_ax, sol.t, sol[2, :])
    lines!(plot.y_ax, sol.t, y_ref, linestyle=:dash)

    lines!(plot.theta_ax, sol.t[:], sol[3, :])
    lines!(plot.theta_ax, sol.t, theta_ref, linestyle=:dash)

    f_1 = sol[7, :]
    f_2 = sol[8, :]

    thrust = f_1 + f_2
    torque = (f_1 - f_2) * 0.1

    lines!(plot.thrust_ax, sol.t, thrust)
    lines!(plot.torque_ax, sol.t, torque)

end

function quad_2d_plot_lsim(t, x, uout)
    fig = Figure(resolution=(1200, 500))

    x_ax = Axis(fig[1, 1], title="x")
    z_ax = Axis(fig[2, 1], title="z")
    theta_ax = Axis(fig[3, 1], title="theta")

    # control input 
    thrust_ax = Axis(fig[1, 2], title="Thrust")
    torque_ax = Axis(fig[2, 2], title="Torque")

    lines!(x_ax, t, x[1, :])
    lines!(z_ax, t, x[2, :])
    lines!(theta_ax, t[:], x[3, :])


    thrust = uout[1, :] + uout[2, :]
    torque = (uout[1, :] - uout[2, :]) * 0.1

    lines!(thrust_ax, t, thrust)
    lines!(torque_ax, t, torque)

    x_ax.ylabel = "x [m]"
    z_ax.ylabel = "z [m]"
    theta_ax.ylabel = "θ [rad]"

    rowgap!(fig.layout, 1)

    fig
end