
export linearize

dynamics!(params) = dynamics!(params[1], params[2])

function linearize(quad_2d::Quad2D, dynamics::Function; control_cmd::Quad2DControlCmd)


    # ForwardDiff.gradient(dynamics!, [quad_2d, control_cmd])
    # f = [0.0, 0.0]
    ForwardDiff.derivative(dynamics, pi / 4)


end


