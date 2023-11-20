module Waste

using FlyingRobots
using Rotations

import FlyingRobots.TrajectoryGeneration: FullyDefined, V3, SingleAxisTrajectory, XYZTrajectory
import FlyingRobots.TrajectoryGeneration: generate_trajectory, get_state
import FlyingRobots.TrajectoryGeneration: Obstacle, SphericalObstacle, PrismObstacle
import FlyingRobots.TrajectoryGeneration: plot_trajectory, plot_obstacle
import FlyingRobots.TrajectoryGeneration: MinimumJerk

# show visualizer (Thread 1)
#plot_elements = FlyingRobots.Gui.show_visualizer()
plot_elements = FlyingRobots.Gui.show_visualizer_only()
vis_ax = plot_elements[:fullscene_visualizer][:axis]

# trjectory duration
T = 4

# initial state
p_0 = V3(0.5, 0, 3)
v_0 = V3(0, 0, 0)
a_0 = V3(0, 0, -9.81)

# final state 
p_f = V3(11.441521789184122, 11.495236168749212, 10.475653181874186)
v_f = V3(0, 0, 0)
a_f = V3(0, 0, -9.81)

mp = FullyDefined(p_0, v_0, a_0, p_f, v_f, a_f)

trajec_xyz = generate_trajectory(MinimumJerk(), mp, T)

# vector of obstacles
obstacles = Obstacle[]

# create spherical obstacle
pos = V3(5, 5, 2.5)
radius = 1.0
spherical_obstacle = SphericalObstacle(pos, V3(), V3(), radius)
push!(obstacles, spherical_obstacle)

# create prism obstacle
dims = V3(5, 5, 8)
pos = V3(7, 1, dims.z / 2)
prism_obstacle = PrismObstacle(pos, one(QuatRotation), dims)
push!(obstacles, prism_obstacle)

# plot trajectory 
@time plot_trajectory(MinimumJerk(), trajec_xyz; ax=vis_ax)

# plot obstacles
plot_obstacle(spherical_obstacle; ax=vis_ax)
plot_obstacle(prism_obstacle; ax=vis_ax)

end