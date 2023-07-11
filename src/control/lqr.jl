
using ControlSystems
using LinearAlgebra
using StaticArrays

function create_lqr_controller(Q::Diagonal, R::Diagonal; params::FrModel, sys::StateSpace)
    K = SMatrix{frmodel_params.nu,frmodel_params.nx}(lqr(sys_d, Q, R))
    dlqr_ctrl = create_frobj(LQRController, K=K)
end

