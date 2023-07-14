using Revise
using LinearAlgebra
using GLMakie
using DifferentialEquations
using StaticArrays
using BenchmarkTools
using ForwardDiff
using NamedTupleTools

using CSV, DataFrames

GLMakie.activate!(inline=false)

# include("types.jl")
# include("utilities.jl")

export setup_control_cb

function setup_control_cb(frmodel::FrModel, quad_obj::Quad2d)
    # Period callback that applies discrete controller 
    control_cb = PeriodicCallback(0.01, initial_affect=true, save_positions=(false, true)) do integrator

        # Extract the parameters
        (; m, l, I_xx, safety_box, K) = integrator.p.quad
        nx, nu, ny, Ts = integrator.p.frmodel
        (; log_matrix) = integrator.p.logger

        # Extract the state 
        X = @view integrator.u[1:nx]

        X_req = generate_trajectory(circle_trajec, quad_obj, integrator.t)

        # compute control input
        X_error = X - X_req
        U = -K * X_error

        # # println("X_req: $(X_req)")
        # #println("X_error: $(X_error)")
        #println("State: $(X)")

        f_1_equilibirum = f_2_equilibirum = -g / 2

        f_1::Float64 = f_1_equilibirum + U[1]
        f_2::Float64 = f_2_equilibirum + U[2]

        # constrain the control input
        f_1 = clamp(f_1, 0.0, 12.5)
        f_2 = clamp(f_2, 0.0, 12.5)

        #Update the control-signal
        integrator.u[nx+1:end] = SA_F64[f_1, f_2]

        # logging
        write_row_vector!(log_matrix, integrator.u, integrator.t, Ts)

    end

    return control_cb
end

function simulate(; X_0, tspan, params, control_cb)

    # setup ODE
    prob = ODEProblem(quad_2d, X_0, tspan, params, callback=control_cb)

    # solve ODE
    # sol = solve(prob, Tsit5(), abstol=1e-8, reltol=1e-8, save_everystep=false, save_on=false);
    sol = solve(prob, Tsit5(), abstol=1e-8, reltol=1e-8, save_everystep=false)
end


#quad_2d_plot_normal(quad2d_plot, sol; y_ref=y_req, z_ref=z_req, theta_ref=θ_req);

# ## benchmarking 
# @btime solve(prob, Tsit5(), abstol=1e-8, reltol=1e-8, save_everystep=false, save_on=false);
# @time quad_2d_plot_normal(quad2d_plot, sol; y_ref=y_req, z_ref=z_req, theta_ref=θ_req);

# ## Trajectory generator benchmarking 

# @time generate_trajectory(circle_trajec, quad_obj, 0.2);

# @time generate_trajectory(circle_trajec, quad_obj, tspan, 0.2);

# n_timesteps = 1000
# @time generate_trajectory(circle_trajec, quad_obj, t_vec);

# # preallocate trajectory matrix
# n_states = 6
# trajectory_matrix = zeros(n_timesteps, n_states)
# t_vec = rand(n_timesteps)
# @time generate_trajectory!(circle_trajec, quad_obj, trajectory_matrix, t_vec);


