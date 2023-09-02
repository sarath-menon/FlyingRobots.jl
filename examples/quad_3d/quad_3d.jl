module quad_3d

using ModelingToolkit
using DiffEqCallbacks
using LinearAlgebra
using OrdinaryDiffEq
using Rotations
using NamedTupleTools
using StaticArrays
using ThreadPools

using GLMakie
using DataFrames
using CSV
using BenchmarkTools
using YAML
import Dates
using ThreadPools
using LabelledArrays

GLMakie.activate!(inline=false)

using MtkLibrary
using FlyingRobots

include("mtk_models.jl")
include("types.jl")
include("logging.jl")

include("sim.jl")
include("modelling.jl")
include("vehicle.jl")

include("computer.jl")
include("controller_utilities.jl")
include("tasks.jl")


include("paths.jl")

include("integrator_callback.jl")

# build system model (Thread 2 )
@time system_build_task = @tspawnat 2 build_system_model()

# # show visualizer (Thread 1)
# plot_elements = FlyingRobots.Gui.show_visualizer()

# create computer 
flight_controller = create_computer("stm32")

#  get result from system build task 
sys, subsystems = fetch(system_build_task)

## Testing
c1 = Channel{ODESolution}(10)
# condition = Threads.Condition()

flag = Observable{Bool}(true)

# prob = sim_setup(sys, subsystems)
# integrator = init(prob, Tsit5(), abstol=1e-8, reltol=1e-8, save_everystep=false)
# df_empty = sim_logging(integrator.sol)

receiver_task_ = @async receiver_task(flag, c1, plot_elements, df_empty)

# # empty DataFrame
# deleteat!(df_empty, :)

#Simulation ----------------------------------------------------
# running vizulizer on 1st thread,(simulator+onboard computer) on 2nd thread
@time sim_task = @tspawnat 2 run_sim_stepping(sys, subsystems, c1, flag; save=false)
df = fetch(sim_task)

flag[] = false

FlyingRobots.Gui.set_2dplot_axislimits(plot_elements; x_low=0, x_high=40, y_max=2)
FlyingRobots.Gui.plot_reset(plot_elements)


function receiver_task(flag, c1, elements, df_empty)

    # delete all existing entries in the dataframe
    deleteat!(df_empty, :)

    # axis limits
    x_range = 20

    x_low = 0
    x_high = x_range

    y_max = ones(3) * 0.1
    y_max_padding = 0.1

    y_local_max = zeros(3)

    # set the initial axis limits
    FlyingRobots.Gui.set_2dplot_axislimits(plot_elements; x_low=x_low, x_high=x_high, y_max=y_max)

    state_plots = elements[:plots_2d][:state_plots]

    #Core.println("Waiting for sol data")
    while true

        # get the latest ODESolution subset from the channel
        sol = take!(c1)

        # convert the ODESolution to a dataframe
        df = sim_logging(sol)

        # append it to the main dataframr
        append!(df_empty, df)

        #compute dynamic axis limits 
        for i = 1:3

            # check the maximum value in the data to be plotted
            y_local_max[i] = maximum(df[!, "(quad1.rb.r(t), $i)"]) + y_max_padding

            # it's it's higher than the current y axis limits, increase the y axis limits
            if y_local_max[i] > y_max[i]
                y_max[i] = y_local_max[i]

                state_plots[i].limits = (x_low, x_high, -y_max[i], y_max[i])
            end
        end

        # check if it's time to change x axis limits
        if df[end, "timestamp"] > x_high

            # set new x_range
            x_low += x_range
            x_high += x_range

            FlyingRobots.Gui.set_2dplot_axislimits(plot_elements; x_low=x_low, x_high=x_high, y_max=y_max)

            # delete data from the prev x axis range since it's not being plotted anymore
            deleteat!(df_empty, :)
        end

        # do the actual plotting
        FlyingRobots.Gui.plot_position_dynamic(elements, df_empty)

        sleep(0.01)

        # ## @show sol.t[end]
        # Core.println(df[!, "timestamp"])

        if flag[] == false
            break
        end
    end

    Core.println("Receiver task done")
end

# plotting ----------------------------------------------------
plot_elements = FlyingRobots.Gui.show_visualizer()

FlyingRobots.Gui.set_sim_instance(plot_elements, df)

FlyingRobots.Gui.plot_position(plot_elements)
FlyingRobots.Gui.plot_orientation(plot_elements)
FlyingRobots.Gui.plot_control_input(plot_elements, motor_thrust_to_body_thrust)

@time flag = FlyingRobots.Gui.start_3d_animation(plot_elements)

reference_generator(Main.quad_3d.Circle_TrajectoryGen(), flight_controller, 10)

scheduler(flight_controller)

Main.quad_3d.Circle_TrajectoryGen == Circle_TrajectoryGen

end

