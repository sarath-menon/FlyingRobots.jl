# # convert standrard quaternion, euler angle to makie quaternin type
function to_makie_quaternion(q::QuatRotation)
    return Quaternionf(q.q.v1, q.q.v2, q.q.v3, q.q.s)
end

function to_std_quaternion(q::Quaternion)
    return QuatRotation(q.q_w, q.q_x, q.q_y, q.q_z)
end

function rotate_mesh(m, q)
    makie_q = to_makie_quaternion(q)
    GLMakie.rotate!(m, makie_q)
end

function get_primary_resolution(index::Int)
    monitors = GLMakie.GLFW.GetMonitors()

    videomode = GLMakie.MonitorProperties(monitors[index]).videomode
    (xscale, yscale) = GLMakie.GLFW.GetMonitorContentScale(monitors[index])

    (height, width) = (videomode.width * xscale, videomode.height * yscale)
    (height, width) = (convert(Int, height), convert(Int, width))
    return (height, width)
end


function sim_logging(sol)
    df = DataFrame(sol)
    # dataframe header formatting
    rename!(df, replace.(names(df), r"getindex" => ""))
    rename!(df, replace.(names(df), r"₊" => "."))

    if save == true
        log_sim(df)
    end

    return df
end