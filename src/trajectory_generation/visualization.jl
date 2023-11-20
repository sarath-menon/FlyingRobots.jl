
# ---------------------------------------------------------------------------------------------------------------------
# Trajectory plotting
# ---------------------------------------------------------------------------------------------------------------------

function plot_trajectory(::MinimumJerk, trajactory::XYZTrajectory; ax, res=Feasible, dt=0.1)
    pos_vec = Vector{Float64}[]
    # vel_vec = Vector{Float64}[]
    # acc_vec = Vector{Float64}[]

    for t = 0:dt:trajactory.T
        pos, vel, acc = get_state(MinimumJerk(), trajactory, t)

        push!(pos_vec, pos)
        # push!(vel_vec, vel)
        # push!(acc_vec, acc)
    end

    # green colour for feasible trajectories
    color = :green

    # red colour for infeasible/indetermiable trajectories
    if res == Infeasible || res == Indeterminable
        color = :red
    end

    # plot position trajectory
    lines!(ax, hcat(pos_vec...), linewidth=5, color=color)
end

# ---------------------------------------------------------------------------------------------------------------------
# Obstacle plotting
# ---------------------------------------------------------------------------------------------------------------------

# spherical obstacle
function plot_obstacle(obstacle::SphericalObstacle; ax)
    mesh!(ax, Sphere(Point3f(obstacle.pos), obstacle.radius), color=:gray80)
end

# prism obstacle
function plot_obstacle(obstacle::PrismObstacle; ax)
    mesh = meshcube(Vec3f(obstacle.pos.x, obstacle.pos.y, obstacle.pos.z - (obstacle.dims.z / 2)), Vec3f(obstacle.dims.x, obstacle.dims.y, obstacle.dims.z), obstacle.dims.x, obstacle.dims.y)
    plane = mesh!(ax, mesh; interpolate=false, color=RGBAf(1, 0, 0, 0.9), transparency=true)
end

# ---------------------------------------------------------------------------------------------------------------------
# 3d box plotting
# ---------------------------------------------------------------------------------------------------------------------

function meshcube(pos, sizexyz, length, width)
    uvs = map(v -> v ./ (3, 2), Vec2f[
        (0, 0), (0, 1), (1, 1), (1, 0),
        (1, 0), (1, 1), (2, 1), (2, 0),
        (2, 0), (2, 1), (3, 1), (3, 0),
        (0, 1), (0, 2), (1, 2), (1, 1),
        (1, 1), (1, 2), (2, 2), (2, 1),
        (2, 1), (2, 2), (3, 2), (3, 1),
    ])
    m = normal_mesh(Rect3f(Vec3f(-length / 2, -width / 2, 0) + pos, sizexyz))
    m = GeometryBasics.Mesh(meta(coordinates(m);
            uv=uvs, normals=normals(m)), faces(m))
end

# ---------------------------------------------------------------------------------------------------------------------
# 2D plane plotting
# ---------------------------------------------------------------------------------------------------------------------

function plot_plane(ax, pos, normal; length=1, width=1)
    plane_mesh = meshcube(Vec3f(0, 0, 0), Vec3f(length, width, 0.01), length, width)
    plane = mesh!(ax, plane_mesh; interpolate=false, color=RGBAf(1, 0, 0, 0.25), transparency=true)

    # move to pos
    GLMakie.translate!(plane, pos)

    # rotate plane to desired  normal
    set_direction_vector!(plane, normal)

    # plot direction vector
    ps = [Point3f(pos)]
    ns = [Point3f(normal)]

    arrows!(ax,
        ps, ns, fxaa=true, # turn on anti-aliasing
        linecolor=:gray, arrowcolor=:black,
        linewidth=0.02, arrowsize=0.1, lengthscale=0.3
    )

    return plane
end


function set_direction_vector!(plane, n_des)

    # get plane orientation as homogeneous transformation matrix
    H = plane.model[]

    # extract rotation matrix
    R = H[1:3, 1:3]

    # get current direction vector 
    n_cur = normalize(V3(R[1, 3], R[2, 3], R[3, 3]))

    # desired direction vector 
    n_des = normalize(n_des)

    # angle between current normal and reqired normal
    α = acos(dot(n_cur, n_des))

    # axis around which this rotation needs to occur in the inertial frame
    rot_axis = normalize(cross(n_cur, n_des))

    # compute attitude error quaternion
    q_w = cos(α / 2)
    q_x = sin(α / 2) * rot_axis.x
    q_y = sin(α / 2) * rot_axis.y
    q_z = sin(α / 2) * rot_axis.z

    q_error = Quaternionf(q_x, q_y, q_z, q_w)

    # apply rotation
    GLMakie.rotate!(plane, q_error)

end

