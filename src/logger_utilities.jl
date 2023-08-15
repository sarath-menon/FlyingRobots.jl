export write!, reset!
export Logger

struct Logger14
    n_fields::Integer
    n_timesteps::Integer
    log_matrix::Matrix{Float64}

    params::Dict{Symbol,Int64}

    function Logger14(; n_fields=n_fields, n_timesteps=n_timesteps)
        # extra column for timestep data
        log_matrix = zeros(n_timesteps, n_fields + 1)

        # parameters
        params = Dict(:current_index => 1)

        new(n_fields, n_timesteps, log_matrix, params)
    end
end

Logger = Logger14

function reset!(logger::Logger)

    # reset logger head
    logger.params[:current_index] = 1

    # set values to zero 
    fill!(logger.log_matrix, 0)
end



function write!(logger::Logger, row::Vector{Float64}, timestep::Float64)

    i = logger.params[:current_index]

    # fill 1st column with the timestep
    logger.log_matrix[i, 1] = timestep

    # next rows - state vector
    for j in 1:logger.n_fields
        logger.log_matrix[i, j+1] = row[j]
    end

    logger.params[:current_index] += 1

    return nothing
end

function Base.read(logger::Logger, index::Integer)
    return logger.log_matrix[index, 2:logger.n_fields+1]
end