
## Dynamics test
module LinearizeTest

using FlyingRobots
using ControlSystems
using StaticArrays
using Test

include("./../examples/quad_2d/quad_2d.jl")
using .Quad2D_Demo

quad_2d_params = (; m=1.0, L=0.1, I_xx=0.003)
L = quad_2d_params.L

initial_state = fr_create(Quad2DState; y=0.0, z=0.0, θ=0.0, ẏ=0.0, ż=0.0, θ̇=0.0)
quad_2d = fr_create(Quad2D; nx=6, nu=2, state=initial_state, params=quad_2d_params)


equilibrium_state = fr_create(Quad2DState; y=0.0, z=1.0, θ=0.0, ẏ=0.0, ż=0.0, θ̇=0.0)
equilibrium_control_cmd = fr_create(Quad2DControlCmd; body_thrust=0.0, body_torque=0.0)



linearize(quad_2d, translational_dynamics, control_cmd=equilibrium_control_cmd)


# testing
function run_tests()
    # quad_2d_params = (; m=1.0, L=0.1, I_xx=0.003)
    # L = quad_2d_params.L

    # initial_state = fr_create(Quad2DState; y=0.0, z=0.0, θ=0.0, ẏ=0.0, ż=0.0, θ̇=0.0)
    # quad_2d = fr_create(Quad2D; nx=6, nu=2, state=initial_state, params=quad_2d_params)
    # new_state = rand(quad_2d.nx)

    # @testset "Type Dynamics: Low level tests" begin
    #     # test update state_func
    #     update_state!(quad_2d, new_state)
    #     @test check_if_struct_equals_vector(quad_2d.state, new_state)
    # end

end

end