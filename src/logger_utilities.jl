
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