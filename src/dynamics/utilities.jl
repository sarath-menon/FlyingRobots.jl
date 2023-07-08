# 2D rotation matrix
R_2D(θ::Float64) = SA_F64[cos(θ) -sin(θ); sin(θ) cos(θ)];

function create_frobj(type_; kwargs...)
    params_tuple = values(kwargs)
    structfromnt(type_, params_tuple)
end

function create_lqr_controller(Q::Diagonal, R::Diagonal; params::FrModel, sys::StateSpace)
    K = SMatrix{frmodel_params.nu,frmodel_params.nx}(lqr(sys_d, Q, R))
    dlqr_ctrl = create_frobj(LQRController, K=K)
end