
export R_2D

# 2D rotation matrix
R_2D(θ::Real) = @SMatrix[cos(θ) -sin(θ); sin(θ) cos(θ)];

