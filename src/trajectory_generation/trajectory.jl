# Trajector generation
function get_trajectory_single_axis(trajec_ax, t)

    α = trajec_ax.α
    β = trajec_ax.β
    γ = trajec_ax.γ
    p_0 = trajec_ax.p_0
    v_0 = trajec_ax.v_0
    a_0 = trajec_ax.a_0

    p = (α / 120) * t^5 + (β / 24) * t^4 + (γ / 6) * t^3 + (a_0 / 2) * t^2 + v_0 / 2 * t + p_0
    v = (α / 24) * t^4 + (β / 6) * t^3 + (γ / 2) * t^2 + a_0 * t + v_0
    a = (α / 6) * t^3 + (β / 2) * t^2 + γ * t + a_0

    return p, v, a
end

function get_trajectory(trajec, t)
    p_x, v_x, a_x = get_trajectory_single_axis(trajec.x, t)
    p_y, v_y, a_y = get_trajectory_single_axis(trajec.y, t)
    p_z, v_z, a_z = get_trajectory_single_axis(trajec.z, t)

    p = V3(p_x, p_y, p_z)
    v = V3(v_x, v_y, v_z)
    a = V3(a_x, a_y, a_z)

    return p, v, a
end