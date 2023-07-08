export quad_2d_plot_normal, quad_2d_plot_lsim

function quad_2d_plot_normal(sol::ODESolution; y_ref, z_ref, theta_ref)
    fig = Figure(resolution=(1200, 600))

    # state variables 
    y_ax = Axis(fig[1, 1], title="y")
    z_ax = Axis(fig[2, 1], title="z")
    theta_ax = Axis(fig[3, 1], title="theta")

    # control input 
    thrust_ax = Axis(fig[1, 2], title="Thrust")
    torque_ax = Axis(fig[2, 2], title="Torque")

    lines!(y_ax, sol.t, sol[1, :])
    lines!(z_ax, sol.t, z_ref, linestyle=:dash)

    lines!(z_ax, sol.t, sol[2, :])
    lines!(y_ax, sol.t, y_ref, linestyle=:dash)

    lines!(theta_ax, sol.t[:], sol[3, :])
    lines!(theta_ax, sol.t, theta_ref, linestyle=:dash)

    f_1 = sol[7, :]
    f_2 = sol[8, :]

    thrust = f_1 + f_2
    torque = (f_1 - f_2) * 0.1

    lines!(thrust_ax, sol.t, thrust)
    lines!(torque_ax, sol.t, torque)

    y_ax.ylabel = "y [m]"
    z_ax.ylabel = "z [m]"
    theta_ax.ylabel = "θ [rad]"

    thrust_ax.ylabel = "Thrust [N]"
    torque_ax.ylabel = "Toruqe [Nm]"


    rowgap!(fig.layout, 1)

    fig
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