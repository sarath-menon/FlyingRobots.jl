
## Logger test
module LoggerTest

using FlyingRobots
using ControlSystems
using StaticArrays
using Test

# logger
n_fields = 4
n_timesteps = 3

logger = Logger(n_fields=n_fields, n_timesteps=n_timesteps)

# data = [1., 2., 3., 4.]
# write!(logger, data, 0.2)
logger.log_matrix

@testset "Logger : log matrix properties " begin

    # no of rows should be = n_timesteps
    @test size(logger.log_matrix)[1] == n_timesteps

    # no of rows should be = (n_fields+1) [extra column for timestep]
    @test size(logger.log_matrix)[2] == n_fields + 1
end


@testset "Logger: operations " begin

    # check if write function works
    reset!(logger)

    data = [1.0, 2.0, 3.0, 4.0]
    write!(logger, data, 0.2)

    @test isequal(read(logger, 1), data)
end



end