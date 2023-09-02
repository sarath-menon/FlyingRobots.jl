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

FlyingRobots.Gui.plot_axis_setup(plot_elements; x_low=0, x_high=40, y_max=2)
FlyingRobots.Gui.plot_reset(plot_elements)


function receiver_task(flag, c1, elements, df_empty)
    deleteat!(df_empty, :)

    # axis limits

    x_range = 20

    x_low = 0
    x_high = x_range

    x_reset_flag = false

    y_max = ones(3) * 0.1
    y_local_max = zeros(3)

    FlyingRobots.Gui.plot_axis_setup(plot_elements; x_low=x_low, x_high=x_high, y_max=y_max[1])

    state_plots = elements[:plots_2d][:state_plots]

    #Core.println("Waiting for sol data")
    while flag[] == true

        sol = take!(c1)

        df = sim_logging(sol)
        append!(df_empty, df)

        #compute dynamic axis limits 

        for i = 1:3

            y_local_max[i] = maximum(df[!, "(quad1.rb.r(t), $i)"])

            if y_local_max[i] > y_max[i]
                y_max[i] = y_local_max[i]

                state_plots[i].limits = (x_low, x_high, -y_max[i], y_max[i])
            end
        end

        if df[end, "timestamp"] > x_high
            # FlyingRobots.Gui.plot_axis_setup(elements; x_low=40, x_high=80, y_max=y_max[1])
            # set new x_range
            x_low += x_range
            x_high += x_range

            for i = 1:3
                state_plots[i].limits = (x_low, x_high, -y_max[i], y_max[i])
            end

            # delete prev plotted data
            deleteat!(df_empty, 1:40)
        end


        FlyingRobots.Gui.plot_position_dynamic(elements, df_empty)
        # break

        sleep(0.01)

        # ## @show sol.t[end]
        # Core.println(df[!, "timestamp"])
    end

    Core.println("Receiver task done")
end


FlyingRobots.Gui.plot_position_dynamic(plot_elements, df_empty)

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

