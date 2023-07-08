export R_2D

# 2D rotation matrix
R_2D(θ::Float64) = SA_F64[cos(θ) -sin(θ); sin(θ) cos(θ)];

function create_safety_box(; x_low, x_high, y_low, y_high, z_low, z_high)
    safety_box = SafetyBox(x_low, x_high, y_low, y_high, z_low, z_high)
    return safety_box
end
