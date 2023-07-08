using DifferentialEquations
##

function lorenz!(du, u, p, t)

    u1 = Ref(u[1])
    u2 = Ref(u[2])
    u3 = Ref(u[3])

    du[1] = 10.0 * (u2[] - u1[])
    du[2] = u1[] * (28.0 - u3[]) - u2[]
    du[3] = u1[] * u2[] - (8 / 3) * u3[]
end

function lorenz2!(du, u, p, t)

    u1 = u[1]
    u2 = u[2]
    u3 = u[3]

    du[1] = 10.0 * (u2[] - u1[])
    du[2] = u1[] * (28.0 - u3[]) - u2[]
    du[3] = u1[] * u2[] - (8 / 3) * u3[]
end

sample_cb = PeriodicCallback(0.01, initial_affect=true) do integrator
    u_modified!(integrator, false)
end

##
u0 = [1.0; 0.0; 0.0]
tspan = (0.0, 1000.0)
prob = ODEProblem(lorenz2!, u0, tspan)
@timev solve(prob, Tsit5(), save_everystep=false);