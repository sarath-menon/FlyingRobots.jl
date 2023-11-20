module Waste

using FlyingRobots
using Rotations
using Distributions

import FlyingRobots.TrajectoryGeneration: FullyDefined, V3, SingleAxisTrajectory, XYZTrajectory
import FlyingRobots.TrajectoryGeneration: generate_trajectory, get_state, collision_checker
import FlyingRobots.TrajectoryGeneration: Obstacle, SphericalObstacle, PrismObstacle
import FlyingRobots.TrajectoryGeneration: plot_trajectory, plot_obstacle
import FlyingRobots.TrajectoryGeneration: MinimumJerk, Feasible, Infeasible

# show visualizer (Thread 1)
#plot_elements = FlyingRobots.Gui.show_visualizer()
plot_elements = FlyingRobots.Gui.show_visualizer_only()
vis_ax = plot_elements[:fullscene_visualizer][:axis]

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

# plot obstacles
plot_obstacle(spherical_obstacle; ax=vis_ax)
plot_obstacle(prism_obstacle; ax=vis_ax)

# trajectory duration
T = 4

# initial state
p_0 = V3(0.5, 0, 4)
v_0 = V3(0, 0, 0)
a_0 = V3(0, 0, -9.81)

# desired final state 
p_f_des = V3(10, 10, 10)
v_f = V3(0, 0, 0)
a_f = V3(0, 0, -9.81)

# allowable final state range (+ or -)
pos_range = V3(2.5, 2.5, 2.5)

n_trajectories = 100

@time for i=1:n_trajectories 
    
    # sample final position from range
    x_rand = rand(Uniform(abs(p_f_des.x) - pos_range.x, abs(p_f_des.x) + pos_range.x))
    y_rand = rand(Uniform(abs(p_f_des.y) - pos_range.y, abs(p_f_des.y) + pos_range.y))
    z_rand = rand(Uniform(abs(p_f_des.z) - pos_range.z, abs(p_f_des.z) + pos_range.z))
    
    p_f = V3(x_rand,y_rand,z_rand)

    mp = FullyDefined(p_0,v_0,a_0,p_f,v_f,a_f)

    trajec_xyz = generate_trajectory(MinimumJerk(), mp, T)
        
    # run collision checker 
    res = Infeasible

    for obstacle in obstacles
        res = collision_checker(trajec_xyz, obstacle;t_min = 0.1)
        # @show typeof(obstacle), res

        if res != Feasible
            break
        end
    end
        
    # plot trajectory 
    plot_trajectory(MinimumJerk(), trajec_xyz; ax=vis_ax, res=res)


end