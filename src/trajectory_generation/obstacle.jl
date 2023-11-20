function collision_checker(trajec, obstacle; t_min, ax=nothing)

    mp = trajec.motion_primitive

    # check if trajec start,end lie inside obstacle
    pos_0 = mp.p_0
    pos_T, _, _ = get_trajectory(trajec, trajec.T)

    if check_if_inside(pos_0, obstacle) || check_if_inside(pos_T, obstacle)
        return Infeasible
    end

    # check - to be done
    return checksection(trajec, obstacle; t_s=0, t_f=trajec.T, t_min=t_min, ax=ax)

end

function checksection(trajec, obstacle; t_s, t_f, t_min, ax=nothing)
    mp = trajec.motion_primitive

    t_split = (t_s + t_f) / 2

    # check if position at time t_split lies inside obstacle
    pos, _, _ = get_trajectory(trajec, t_split)

    if check_if_inside(pos, obstacle)
        return Infeasible

        # t_min limits recursive depth of checker 
    elseif t_f - t_s < t_min
        @show t_f - t_s
        return Indeterminable
    end

    # find plane separating pos(t_split) and obstacle
    n, boundary_point = get_tangent_plane(pos, obstacle)

    # plotting
    if ax != nothing
        # plot seperator plane
        plot_plane(ax, boundary_point, n; length=1, width=3)

        # plot critical point
        scatter!(Point3f(pos))

        # pause to observe animation
        sleep(2) #seconds
    end

    # compute critical points of the position trajectory from t_split to t_f
    c_0 = n' * mp.v_0
    c_1 = n' * mp.a_0
    c_2 = (1 / 2) * n' * [trajec.x.γ; trajec.y.γ; trajec.z.γ]
    c_3 = (1 / 6) * n' * [trajec.x.β; trajec.y.β; trajec.z.β]
    c_4 = (1 / 24) * n' * [trajec.x.α; trajec.y.α; trajec.z.α]

    # construct polynomial from coeff and compute roots (critical points)
    vel_polynomial = Polynomial([c_0, c_1, c_2, c_3, c_4])
    crit_points = roots(vel_polynomial)

    # discard complex roots
    crit_points = discard_imag_vals(crit_points)

    # discard roots outside the range (t_s, t_f) (also remove critical points, add later - numerics)
    filter!(x -> x > t_s && x < t_f, crit_points)

    # add t_s, t_f back  
    push!(crit_points, t_s)
    push!(crit_points, t_f)

    # critical points in (t_split, t_f]
    crit_points_first = filter(t -> t > t_split && t <= t_f, crit_points)

    # sort them in ascending order
    sort!(crit_points_first)

    t_prev = t_split
    for t in crit_points_first

        # pos at critical point 
        pos, _, _ = get_trajectory(trajec, t)

        # check if pos at critical lies on obstacle side of the plane ,ie, if dist is negative
        dist = n' * (pos - boundary_point)

        if dist < 0
            result = checksection(trajec, obstacle; t_s=t_prev, t_f=t_f, t_min=t_min, ax=ax)
            if result == Feasible
                break
            else
                return result
            end
        end

        t_prev = t
    end


    # critical points from (t_s, t_split]
    crit_points_second = filter(t -> t >= t_s && t < t_split, crit_points)

    # sort them in descending order
    sort!(crit_points_second, rev=true)

    t_prev = t_split
    for t in crit_points_second

        # pos at critical point 
        pos, _, _ = get_trajectory(trajec, t)

        # check if pos at critical lies on obstacle side of the plane ,ie, if dist is negative
        dist = n' * (pos - boundary_point)

        if dist < 0
            return checksection(trajec, obstacle; t_s=t_s, t_f=t_prev, t_min=t_min, ax=ax)
        end

        t_prev = t
    end

    return Feasible
end

# to check if a point p in global frame is inside a spherical obstacle
function check_if_inside(p, obstacle::SphericalObstacle)
    return norm(p - obstacle.pos) <= obstacle.radius
end

# to check if a point p in global frame is inside a prism obstacle
function check_if_inside(p, obstacle::PrismObstacle)
    # transform p to local frame of prism
    p_local = inv(obstacle.orientation) * (p - obstacle.pos)

    # check if p (local) is inside the prism
    is_inside = (abs(p_local.x) <= obstacle.dims.x / 2) && (abs(p_local.y) <= obstacle.dims.y / 2) && (abs(p_local.z) <= obstacle.dims.z / 2)

    return is_inside
end

# to get distance to obstacle
get_obstacle_distance(p, obstacle) = norm(p - obstacle.pos)