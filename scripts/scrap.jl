module Scrap

using GLMakie
using DataStructures

GLMakie.activate!(inline=false)

fig = Figure()

x_low = 0
x_high = 10

ax = Axis(fig[1, 1], limits=(x_low, x_high, -1, 1))

# plot buffer 
resolution = 0.1
x_range = Int((x_high - x_low) / resolution) + 1

time_cb = CircularDeque{Float64}(x_range)
data_cb = CircularDeque{Float64}(x_range)
# time_cb = CircularBuffer{Float64}(x_range)
# data_cb = CircularBuffer{Float64}(x_range)

time_cb_obs = Observable{Vector{Float64}}(time_cb.buffer)
data_cb_obs = Observable{Vector{Float64}}(data_cb.buffer)


scatter!(ax, time_cb_obs, data_cb_obs, color=:black, markersize=15)

t_range = 0:0.1:20

function reset_plot()
    fill!(time_cb_obs[], 0)
    fill!(data_cb_obs[], 0)

    fill!(time_cb, 0)
    fill!(data_cb, 0)

    notify(time_cb_obs)
    notify(data_cb_obs)

    ax.limits = (x_low, x_high, -1, 1)
end

# animation
@time for (i, t) in enumerate(t_range)

    # computation 
    y = cos(t)

    # push data to buffers
    push!(time_cb, t)
    push!(data_cb, y)

    if i % 10 == 0

        data_cb_obs[] = data_cb.buffer
        time_cb_obs[] = time_cb.buffer
    end

    x_min = time_cb[1]

    if time_cb[end] <= 10
        x_max = 10
    else
        x_max = time_cb[end]
    end

    ax.limits = (x_min, x_max, -1, 1)

    sleep(0.01)
end

reset_plot()

function reset_plot()
    fill!(time_cb_obs[], 0)
    fill!(data_cb_obs[], 0)

    empty!(time_cb)
    empty!(data_cb)

    notify(time_cb_obs)
    notify(data_cb_obs)

    ax.limits = (x_low, x_high, -1, 1)
end

reset_plot()
# animation
@time for (i, t) in enumerate(t_range)

    # computation 
    y = cos(t)

    if i > x_range
        pop!(time_cb)
        pop!(data_cb)
    end

    # push data to buffers
    @async pushfirst!(time_cb, t)
    @async pushfirst!(data_cb, y)

    if i % 4 == 0
        time_cb_obs[] = time_cb.buffer
        data_cb_obs[] = data_cb.buffer
    end

    x_min = last(time_cb)

    if first(time_cb) <= 10
        x_min = 0
        x_max = 10
    else
        x_max = first(time_cb)
    end

    @async if i % 10 == 0
        ax.limits = (x_min, x_max, -1, 1)
    end

    sleep(0.01)
end

reset_plot()

scatter!(ax, time_cb_obs, data_cb_obs, color=:black)

points = Observable(Point2f[(0, 0)])
scatter!(ax, points, color=:black)

@time for (i, t) in enumerate(t_range)

    # computation 
    y = cos(t)

    # push data to buffers
    new_point = Point2f(t, y)
    points[] = push!(points[], new_point)

    if t <= 10
        x_min = 0
        x_max = 10
    else
        x_min = t - 10
        x_max = t
    end

    if i % 10 == 0
        ax.limits = (x_min, x_max, -1, 1)
    end

    sleep(0.01)
end

points[] = Float64[]

display(fig)


end