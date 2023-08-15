export write!
export Logger

mutable struct Logger1
    n_fields::Integer
    n_timesteps::Integer
    log_matrix::Matrix{Float64}

    current_index::Integer

    function Logger1(; n_fields=n_fields, n_timesteps=n_timesteps)
        # extra column for timestep data
        log_matrix = zeros(n_timesteps, n_fields + 1)

        # initialize index  
        current_index = 1

        new(n_fields, n_timesteps, log_matrix, current_index)
    end
end

Logger = Logger1


function write!(logger::Logger, row::Vector{Float64}, timestep::Float64, Ts::Float64; start_index=0)
    # find logging index
    n_timestep = convert(Int, round(timestep / Ts) + 1)

    # index = n_timestep + start_index
    i = logger.current_index

    # fill 1st column with the timestep
    logger.log_matrix[i, 1] = timestep

    # next rows - state vector
    for j in 1:8
        logger.log_matrix[i, j+1] = row[j]
    end

    logger.current_index += 1

    return nothing
end


function write!(logger::Logger, row::Vector{Float64}, timestep::Float64)
    # index = n_timestep + start_index
    i = logger.current_index

    # fill 1st column with the timestep
    logger.log_matrix[i, 1] = timestep

    # next rows - state vector
    for j in 1:8
        logger.log_matrix[i, j+1] = row[j]
    end

    logger.current_index += 1

    return nothing
end



function write!(logger::Logger, row::Vector{Float64}; Ts::Float64=0.01, timestep::Float64=0.0, initial_condition::Bool=false, offset::Int=0)

    if initial_condition == true
        n_timestep = 1
        timestep = 0.0
    else
        offset = 1
    end

    # find logging index
    n_timestep = convert(Int, round(timestep / Ts) + 1)
    index = n_timestep + offset

    A = logger.log_matrix

    # fill 1st column with the timestep
    A[index, 1] = timestep

    # next rows - state vector
    for j in 1:8
        A[index, j+1] = row[j]
    end
end