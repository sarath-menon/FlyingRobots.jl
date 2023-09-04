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
using DataStructures

GLMakie.activate!(inline=false)

using MtkLibrary
using FlyingRobots

using FlyingRobots.Gui: SimAccMode, RealtimeSim, AcceleratedSim

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
#df_empty = get_empty_df(sys, subsystems)
c1 = Channel{ODESolution}(10)

sim_state = Observable{Bool}(true)
sim_acc_state = Observable{SimAccMode}()

let
    obs_func = on(sim_state, weak=true) do val
        if sim_state[] == true

            if sim_acc_state[] == RealtimeSim()
                FlyingRobots.Gui.plot_reset(plot_elements)
                gui_dynamic_plotter_task = @async FlyingRobots.Gui.gui_dynamic_plotter(plot_elements, c1, df_empty)
                @time sim_task = @tspawnat 2 run_sim_stepping(sys, subsystems, c1, sim_state, sim_acc_state; save=false)
                Core.println("Starting Realtime simulation")


            elseif sim_acc_state[] == AcceleratedSim()
                @time sim_task = @tspawnat 2 run_sim_stepping(sys, subsystems, c1, sim_state, sim_acc_state; save=false)
                Core.println("Starting Accelerated simulation")

                df = fetch(sim_task)
                FlyingRobots.Gui.set_sim_instance(plot_elements, df)
                FlyingRobots.Gui.plot_reset(plot_elements)
                FlyingRobots.Gui.plot_position(plot_elements)
            end
        end

    end

end



obs_func = nothing

#Simulation ----------------------------------------------------
# running vizulizer on 1st thread,(simulator+onboard computer) on 2nd thread
# @time sim_task = @tspawnat 2 run_sim_stepping(sys, subsystems, c1, sim_state; save=false)
@time sim_task = @async run_sim_stepping(sys, subsystems, c1, sim_state, sim_acc_state; save=false)
df = fetch(sim_task)

sim_state[] = false
sim_state[] = true

# plotting ----------------------------------------------------
plot_elements = FlyingRobots.Gui.show_visualizer()
FlyingRobots.Gui.plot_reset(plot_elements)

# connect observables
connect!(sim_state, plot_elements[:sim_state])
connect!(sim_acc_state, plot_elements[:sim_acc_state])

FlyingRobots.Gui.set_sim_instance(plot_elements, df)
# FlyingRobots.Gui.set_sim_flag(plot_elements, sim_state)


FlyingRobots.Gui.plot_position(plot_elements)
FlyingRobots.Gui.plot_orientation(plot_elements)
FlyingRobots.Gui.plot_control_input(plot_elements, motor_thrust_to_body_thrust)

@time sim_state = FlyingRobots.Gui.start_3d_animation(plot_elements)

end

