## Trajectory generator test 



tspan = (0.0, 100.0)
dt = frmodel_params.Ts

circle_trajec = create_frobj(CircleTrajectory; r=1.0, ω=0.1 * π, y₀=2.0, z₀=2.0)
quad_params = (; m=1.0)

(y_vec, z_vec, θ_vec, ẏ_vec, ż_vec, θ̇_vec) = generate_trajectory(circle_trajec, quad_params, tspan, dt);

# plot trajectory
fig = Figure()
ax = Axis(fig[1, 1],
    aspect=1,
    title="Circular trajecory in the Y-Z Plane",
    xlabel="Y Axis",
    ylabel="Z Axis"
)
lines!(ax, y_vec, z_vec)

display(fig)