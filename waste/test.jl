using DifferentialEquations
using BenchmarkTools
using CSV, DataFrames
using StaticArrays
##

function lorenz!(du, u, p, t)

    u1 = Ref(u[1])
    u2 = Ref(u[2])
    u3 = Ref(u[3])

    du[1] = 10.0 * (u2[] - u1[])
    du[2] = u1[] * (28.0 - u3[]) - u2[]
    du[3] = u1[] * u2[] - (8 / 3) * u3[]
end

# # run at every timestep
# condition(u, t, integrator) = true

function affect!(integrator)

    # # Extract the state 
    # X::Vector{Float64} = integrator.u[1:3]

    # push!(logging_vars, (integrator.t, integrator.u))
end

cb = DiscreteCallback(condition, affect!)

# sample_cb = PeriodicCallback(0.01, initial_affect=true, save_positions=(false, true)) do integrator
#     # u_modified!(integrator, false)
# end

##
u0 = [1.0; 0.0; 0.0]
tspan = (0.0, 60.0)

## Logging variables
#logging_vars = DataFrame(timestep=Float64[], X=Vector{Float64}[])

prob = ODEProblem(lorenz!, u0, tspan)
@timev sol = solve(prob, Tsit5(), dt=0.01, adaptive=false, dense=false);

df = DataFrame(sol)
CSV.write("logs/log_test.csv", df)
## Benchmarking 

# save every step disabled
@btime sol = solve(prob, Tsit5(), save_everystep=false, adaptive=false, dense=false, dt=0.01);

# save every step enbled
@btime sol = solve(prob, Tsit5(), adaptive=false, dense=false, dt=0.01);

#@profview solve(prob, Tsit5(), save_everystep=false);