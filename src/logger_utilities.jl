export write!
export Logger

mutable struct Logger7
    n_fields::Integer
    n_timesteps::Integer
    log_matrix::Matrix{Float64}

    function Logger7(; n_fields=n_fields, n_timesteps=n_timesteps)
        # extra column for timestep data
        log_matrix = zeros(n_timesteps, n_fields + 1)
        new(n_fields, n_timesteps, log_matrix)
    end
end

Logger = Logger7


function write!(logger::Logger, row::Vector{Float64}, timestep::Float64, Ts::Float64; start_index=0)
    # find logging index
    n_timestep = convert(Int, round(timestep / Ts) + 1)
    index = n_timestep + start_index

    # fill 1st column with the timestep
    logger.log_matrix[index, 1] = timestep

    # next rows - state vector
    for j in 1:8
        logger.log_matrix[index, j+1] = row[j]
    end

    return nothing
end

function write!(log_matrix::Matrix, row::Vector{Float64}, timestep::Float64, Ts::Float64; start_index=0)
    # find logging index
    n_timestep = convert(Int, round(timestep / Ts) + 1)
    index = n_timestep + start_index

    # fill 1st column with the timestep
    log_matrix[index, 1] = timestep

    # next rows - state vector
    for j in 1:8
        log_matrix[index, j+1] = row[j]
    end

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