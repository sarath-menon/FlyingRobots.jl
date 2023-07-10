function generate_trajectory(trajec_params::CircleTrajectory, quad::Quad2d, t::Float64)
    # Extract the parameters
    m = quad.m

    r::Float64 = trajec_params.r
    ω::Float64 = trajec_params.ω

    y₀ = trajec_params.y₀
    z₀ = trajec_params.z₀

    y::Float64 = (r * cos(ω * t)) + y₀
    z::Float64 = (r * sin(ω * t)) + z₀

    ẏ::Float64 = (-r * sin(ω * t)) * ω
    ż::Float64 = (r * cos(ω * t)) * ω

    ÿ::Float64 = -y * (ω^2)
    z̈::Float64 = -z * (ω^2)


    ÿ_func(r::Real, ω::Real, t::Real) = -r * cos(ω * t) * (ω^2)
    z̈_func(r::Real, ω::Real, t::Real) = -r * sin(ω * t) * (ω^2)
    θ_func(t::Real) = atan(-m * ÿ_func(r, ω, t), m * (z̈_func(r, ω, t) - g))

    # compute θ, θ̇ using differential flatness
    θ::Float64 = θ_func(t)
    θ̇::Float64 = ForwardDiff.derivative(θ_func, t) # constant value (precomputed)

    return @SVector [y, z, θ, ẏ, ż, θ̇]
end

function generate_trajectory(trajec_params::CircleTrajectory, quad::Quad2d, tspan::Tuple, dt::Float64)
    y_vec = Float64[]
    z_vec = Float64[]
    θ_vec = Float64[]
    ẏ_vec = Float64[]
    ż_vec = Float64[]
    θ̇_vec = Float64[]

    for t in tspan[1]:dt:tspan[2]
        (y, z, θ, ẏ, ż, θ̇) = generate_trajectory(trajec_params, quad, t)
        push!(y_vec, y)
        push!(z_vec, z)
        push!(θ_vec, θ)
        push!(ẏ_vec, ẏ)
        push!(ż_vec, ż)
        push!(θ̇_vec, θ̇)
    end

    return [y_vec, z_vec, θ_vec, ẏ_vec, ż_vec, θ̇_vec]
end

function generate_trajectory(trajec_params::CircleTrajectory, quad::Quad2d, t_vec::Vector{Float64})
    y_vec = Float64[]
    z_vec = Float64[]
    θ_vec = Float64[]
    ẏ_vec = Float64[]
    ż_vec = Float64[]
    θ̇_vec = Float64[]

    for t in t_vec
        (y, z, θ, ẏ, ż, θ̇) = generate_trajectory(trajec_params, quad, t)
        push!(y_vec, y)
        push!(z_vec, z)
        push!(θ_vec, θ)
        push!(ẏ_vec, ẏ)
        push!(ż_vec, ż)
        push!(θ̇_vec, θ̇)
    end

    return [y_vec, z_vec, θ_vec, ẏ_vec, ż_vec, θ̇_vec]
end

function generate_trajectory!(trajec_params::CircleTrajectory, quad::Quad2d, log_matrix::Matrix{Float64}, t_vec::Vector{Float64})

    for (i, t) in enumerate(t_vec)
        (y, z, θ, ẏ, ż, θ̇) = generate_trajectory(trajec_params, quad, t)
        log_matrix[i, 1] = y
        log_matrix[i, 2] = z
        log_matrix[i, 3] = θ
        log_matrix[i, 4] = ẏ
        log_matrix[i, 5] = ż
        log_matrix[i, 6] = θ̇
    end
end

