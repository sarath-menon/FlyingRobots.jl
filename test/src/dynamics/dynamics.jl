module ComputerTest

using FlyingRobots
using Rotations
using FlyingRobots.Dynamics: quaternion_integrator!, quaternion_integrator

# returning function test 
q0 = one(QuatRotation)
ω = [1 0 0]
dt = 0.01

q1 = quaternion_integrator(q0, ω, dt)

# quaternion form 
@show q1.q

# ZYX Euler angle form
@show Rotations.params(RotZYX(q1))


# in-place function test 
q = one(QuatRotation)
ω = [1.0 0 0]
dt = 0.01

quaternion_integrator!(q, ω, dt)

# quaternion form 
@show q1.q

# ZYX Euler angle form
@show Rotations.params(RotZYX(q1))


end