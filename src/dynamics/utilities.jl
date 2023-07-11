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


function write_row_vector!(A, i, row)
    # log timestep
    A[i, 1] = i

    # log dat
    for j in 1:8
        A[i+1, j] = row[j]
    end
end

function write_row_vector!(A, row::Vector{Float64}, timestep::Float64, Ts::Float64)

    # find logging index
    n_timestep = convert(Int, round(timestep / Ts) + 1)

    # log timestep
    A[n_timestep, 1] = timestep

    # log dat
    for j in 1:8
        A[n_timestep+1, j] = row[j]
    end
end