
# get tangent plane seperating spherical obstacle and a point p oustide the obstacle
# plane is defined by a normal vectory and point
function get_tangent_plane(p, obstacle::SphericalObstacle)
    normal_vec = normalize(p - obstacle.pos)

    # intersection point btw plane and boundary of obstacle
    boundary_point = (normal_vec * obstacle.radius) + obstacle.pos

    return normal_vec, boundary_point
end

function get_tangent_plane(p, obstacle::PrismObstacle)
    # transform p to local frame of prism
    p_local = inv(obstacle.orientation) * (p - obstacle.pos)

    # Compute Euclidean projection point on cube
    proj = [0.0, 0.0, 0.0]

    for i = 1:3
        proj[i] = clamp(p_local[i], -obstacle.dims[i] / 2, obstacle.dims[i] / 2)
    end

    # transform projection to to global frame
    boundary_point = obstacle.orientation * proj + obstacle.pos

    normal_vec = normalize(p - boundary_point)

    return normal_vec, boundary_point
end

# discard imaginary values from a vector
function discard_imag_vals(V)
    V_real = real(V)
    V_imag = imag(V)

    for i = 1:length(V)
        if V_imag[i] != 0
            V_real[i] = 0
        end
    end

    return V_real
end