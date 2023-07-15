export write_row_vector!

function write_row_vector!(A, i, row)
    # log timestep
    A[i, 1] = i

    # log dat
    for j in 1:8
        A[i+1, j] = row[j]
    end
end

function write_row_vector!(A, row::Vector{Float64}, timestep::Float64, Ts::Float64; start_index=0)
    # find logging index
    n_timestep = convert(Int, round(timestep / Ts) + 1)
    index = n_timestep + start_index

    # fill 1st column with the timestep
    A[index, 1] = timestep

    # next rows - state vector
    for j in 1:8
        A[index, j+1] = row[j]
    end
end