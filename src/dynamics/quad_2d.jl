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


# 




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


