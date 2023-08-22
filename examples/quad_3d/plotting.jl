
function plot_position_attitude(sol::ODESolution)
    fig = Figure(resolution=(1000, 700))

    ax1 = Axis(fig[1, 1], title="x Position")
    ax2 = Axis(fig[1, 2], title="y Position")
    ax3 = Axis(fig[1, 3], title="z Position")

    # ax2 = Axis(fig[1, 2], title="Velocity")

    ax4 = Axis(fig[2, 1], title="Attitude (quaternion)")
    ax5 = Axis(fig[2, 2], title="Angular velocity")

    lines!(ax1, sol.t, sol[plant.r[1]])
    lines!(ax2, sol.t, sol[plant.r[2]])
    lines!(ax3, sol.t, sol[plant.r[3]])

    # lines!(ax2, sol.t, sol[plant.ṙ[1]])
    # lines!(ax2, sol.t, sol[plant.ṙ[2]])
    # lines!(ax2, sol.t, sol[plant.ṙ[3]])

    lines!(ax4, sol.t, sol[plant.q[1]])
    lines!(ax4, sol.t, sol[plant.q[2]])
    lines!(ax4, sol.t, sol[plant.q[3]])
    lines!(ax4, sol.t, sol[plant.q[4]])

    lines!(ax5, sol.t, sol[plant.ω[1]])
    lines!(ax5, sol.t, sol[plant.ω[2]])
    lines!(ax5, sol.t, sol[plant.ω[3]])

    display(fig)
end