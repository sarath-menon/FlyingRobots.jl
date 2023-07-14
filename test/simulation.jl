module SimulationTest

using FlyingRobots
using ControlSystems
using StaticArrays
using Test
using BenchmarkTools
using DifferentialEquations

include("./../examples/quad_2d/quad_2d.jl")

quad_2d_params = (; m=1.0, L=0.1, I_xx=0.003)
# initial_state = fr_create(Quad2DState; y=0.0, z=0.0, θ=0.0, ẏ=0.0, ż=0.0, θ̇=0.0)
quad_2d = fr_create(Quad2D; nx=6, nu=2, state=initial_state, params=quad_2d_params)

# setup ODE
tspan = (0.0, 1.0)
state = zeros(8)
prob = ODEProblem(dynamics4_diffeq, state, tspan, quad_2d_params)

# solve ODE
# sol = solve(prob, Tsit5(), abstol=1e-8, reltol=1e-8, save_everystep=false, save_on=false);
sol = solve(prob, Tsit5(), abstol=1e-8, reltol=1e-8, save_everystep=false)


sol.t

# Benchmarking 

quad_2d_params = (; m=1.0, L=0.1, I_xx=0.003)
initial_state = fr_create(Quad2DState; y=0.0, z=0.0, θ=0.0, ẏ=0.0, ż=0.0, θ̇=0.0)
quad_2d = fr_create(Quad2D; nx=6, nu=2, state=initial_state, params=quad_2d_params)
ctrl_cmd = fr_create(Quad2DControlCmd; body_thrust=-g, body_torque=0.0)

# core dynamics function 
@time dynamics2!(initial_state, ctrl_cmd, quad_2d_params)

state = zeros(8)
d_state = zeros(8)

##= Wrapper around core dynamics function for DifferentialEquations.jl solver =#
@btime dynamics4_diffeq(d_state, state, quad_2d_params, 0.01)

##= DifferentialEquations.jl solver =#
@btime sol = solve(prob, Tsit5(), abstol=1e-8, reltol=1e-8, save_everystep=false)

end