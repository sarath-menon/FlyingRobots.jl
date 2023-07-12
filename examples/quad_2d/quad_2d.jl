module Quad2D_Demo

using FlyingRobots

using StaticArrays

include("types.jl")
include("dynamics.jl")


quad_2d_params = (; m=1.0, L=0.1, I_xx=0.003)
initial_state = Quad2DState(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
initial_ctrl_cmd = Quad2DActuatorCmd(0.0, 0.0)
quad_2d = Quad2D(6, 2, initial_state, quad_2d_params)

diffeq_state = convert_Fr_types_to_diffeq_state(initial_state, initial_ctrl_cmd)

update_state!(quad_2d, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

end