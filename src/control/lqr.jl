
using ControlSystems
using LinearAlgebra
using StaticArrays


#--------------------------------------------------------------------------

struct LQRController5
    K::SMatrix
    k_f::Vector{Float64}
    u_equilibrium::Vector{Float64}
end

LQRController = LQRController5



#--------------------------------------------------------------------------


function create_lqr_controller(Q::Diagonal, R::Diagonal; params::FrModel, sys::StateSpace, k_f::Vector{Float64}=[0.0])
    # LQR gain matrix K
    K = SMatrix{params.nu,params.nx}(lqr(sys, Q, R))

    # calculate equilibium control action 
    thrust_equilibirum::Float64 = -g

    f_1_equilibirum = thrust_equilibirum / 2
    f_2_equilibirum = thrust_equilibirum / 2

    dlqr_ctrl = fr_create(LQRController, K=K, k_f=k_f, u_equilibrium=[f_1_equilibirum, f_2_equilibirum])
end

apply_lqr_ctrl(ctrl::LQRController, X_error) = (-ctrl.K * X_error) + ctrl.k_f + u_equilibrium

