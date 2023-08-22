module Scrap

include("./../examples/quad_2d/quad_2d.jl")

X₀ = Quad2DState(3, 1, 0, 0, 0, 0)
U₀ = Quad2DActuatorCmd(0, 0)

values = [Quad2DState, Quad2DActuatorCmd]
keys = collect(1:1:length(list1))

dict1 = Dict(zip(keys, values))
sort(dict1)



end