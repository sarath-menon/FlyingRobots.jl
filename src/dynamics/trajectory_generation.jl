function generate_circle_trajectory(trajec_params::NamedTuple, quad_params::NamedTuple, t::Float64)
    m::Float64 = quad_params.m

    r::Float64 = trajec_params.r
    ω::Float64 = trajec_params.ω
    g::Float64 = trajec_params.g

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

    return [y, z, θ, ẏ, ż, θ̇]
end

function generate_circle_trajectory(trajec_params::NamedTuple, quad_params::NamedTuple, tspan::Tuple, dt::Float64)
    y_vec = Float64[]
    z_vec = Float64[]
    θ_vec = Float64[]
    ẏ_vec = Float64[]
    ż_vec = Float64[]
    θ̇_vec = Float64[]

    for t in tspan[1]:dt:tspan[2]
        (y, z, θ, ẏ, ż, θ̇) = generate_circle_trajectory(trajec_params, quad_params, t)
        push!(y_vec, y)
        push!(z_vec, z)
        push!(θ_vec, θ)
        push!(ẏ_vec, ẏ)
        push!(ż_vec, ż)
        push!(θ̇_vec, θ̇)
    end

    return [y_vec, z_vec, θ_vec, ẏ_vec, ż_vec, θ̇_vec]
end

function generate_circle_trajectory(trajec_params::NamedTuple, quad_params::NamedTuple, t_vec::Vector{Float64})
    y_vec = Float64[]
    z_vec = Float64[]
    θ_vec = Float64[]
    ẏ_vec = Float64[]
    ż_vec = Float64[]
    θ̇_vec = Float64[]

    for t in t_vec
        (y, z, θ, ẏ, ż, θ̇) = generate_circle_trajectory(trajec_params, quad_params, t)
        push!(y_vec, y)
        push!(z_vec, z)
        push!(θ_vec, θ)
        push!(ẏ_vec, ẏ)
        push!(ż_vec, ż)
        push!(θ̇_vec, θ̇)
    end

    return [y_vec, z_vec, θ_vec, ẏ_vec, ż_vec, θ̇_vec]
end
