# Forward-simulation based planning for robotic exploration
The software contained in the ase_exploration package is a [ROS](www.ros.org) integrated planner for robotic exploration tasks, taking as input data from various costmaps and outputting new exploration targets for the robot.
To execute the exploration task, this software uses [move_base](http://wiki.ros.org/move_base).
In order to use this software, you will need to have properly configured move_base to run on your robot.

The planner software uses layers of costmaps to track free, occupied, and unknown space.
Based on this information, it will sample possible trajectories and sensor readings, and iteratively improve the trajectories to maximize the mutual information of the map and future observations.
Based on a few heuristic rules, the planner detects whether the robot might be stuck or if the local trajectories it has sampled are not very informative.
If this happens, the planner will instead apply classical frontier-based exploration to select the next exploration target, and afterwards again resumes planning via forward simulation of local trajectories.
For technical details on the approach, please refer to the publication listed below.

The basic interface is through ROS actions - as long as an request for exploration provided in the form of an action is active, the planner will keep producing new exploration targets.
The process is stopped when an error occurs, or when the user cancels the action.
The software will pass any exploration targets it computes to move_base via a move_base_msgs::MoveBaseAction type action.

# Video and publication

Click the link below to view a video showing the software in action:
[![Planning for robotic exploration based on forward simulation](http://i.imgur.com/AuwCkYr.png)](https://www.youtube.com/watch?v=-7c_RaZ-KTU)

Further details are available in our journal paper on the subject.
If you use this code in an academic context, please cite the paper:

Mikko Lauri, Risto Ritala. *Planning for robotic exploration based on forward simulation*, Robotics and Autonomous Systems, Vol 83, (2016), pp. 15-31, DOI: [10.1016/j.robot.2016.06.008](https://doi.org/10.1016/j.robot.2016.06.008)

A preprint version is also [available on arXiv](https://arxiv.org/abs/1502.02474).

BiBTeX:
```
@article{Lauri2016planning,
  author  = {Mikko Lauri and Risto Ritala}, 
  title   = {Planning for robotic exploration based on forward simulation},
  journal = {Robotics and Autonomous Systems},
  year    = 2016,
  volume  = 83,
  pages   = {15 - 31},
  doi     = {10.1016/j.robot.2016.06.008}
}
```

### Acknowledgment
As implementation of the fallback exploration strategy based on classical frontier exploration, we apply Paul Bovbel's [frontier_exploration](https://github.com/paulbovbel/frontier_exploration) package.
This package also in part inspired the implementation our planner.

# Installation
The code is hosted on [GitHub](https://github.com/laurimi/ase_exploration).
The package has been tested to work with ROS Indigo, running on Ubuntu 14.04, using gcc 4.8.4 as compiler.
To build, simply clone this repository to your catkin workspace `src` directory, and run `catkin_make` in the workspace main directory.

For improved performance, this package uses OpenMP for evaluating trajectories in parallel.
C++11 features are used in the code, and support is required from the compiler.


# Simulator demo: Turtlebot exploring an environment
You can test the exploration planner by running it using simulator.
Here, we apply mostly standard settings from the [turtlebot_stage](http://wiki.ros.org/turtlebot_stage) package.
To run the demo, you need to install the dependencies:
```
sudo apt-get install ros-indigo-turtlebot-simulator ros-indigo-frontier-exploration ros-indigo-turtlebot-navigation
```

Before continuing, make sure you have sourced `setup.bash` of the catkin workspace you installed the exploration package in, do this as follows (changing the path appropriately)
```
source ~/catkin_ws/devel/setup.bash
```

Now run the main launch file:
```
roslaunch ase_exploration simulator_exploration.launch
```
The simulator will now start, along with other nodes and an rviz visualization.

Finally, start the action client to send an exploration command to the robot.
**This command must be run in a terminal window where `setup.bash` of the catkin workspace you installed the exploration package in has been sourced.**
```
rosrun actionlib axclient.py /exploration
```
Click on "Send", and the simulated Turtlebot will start exploring the environment.

# ROS Node details: ase_exploration_planner_node

## 1. Actions provided
This node provides an implementation of the SimpleActionServer for the ExploreAction type defined in this package.
This action is provided with the name `exploration`.
Your best bet to get started is probably to use actionlib's axclient to send an action request to start exploration: `rosrun actionlib axclient.py /exploration`.

### ExploreAction
ExploreAction is very simple. It has an empty goal definition, and an empty result definition, and as feedback will output the current exploration target produced by the planner.
Once the user gives the node a exploration task through this action, the planner will produce exploration targets until an error occurs or the user cancels the action.

## 2. Actions called
The exploration targets produced by the planner are sent as navigation goals to move_base.
This is handled by a SimpleActionClient calling an move_base_msgs::MoveBaseAction on the topic `move_base`.

## 3. Services required
This node requires and calls two services from the [frontier_exploration](https://github.com/paulbovbel/frontier_exploration) package.
They are:

- `frontier_exploration/UpdateBoundaryPolygon`
- `frontier_exploration/GetNextFrontier`

In practice you will need to launch an instance of `frontier_exploration` and set its parameters according to the behaviour you want in case this exploration node has to fallback to using frontier exploration.
The `ase_exploration_node` will call services from `frontier_exploration` as required.

## 4. Subscribed topics
The node requires two types of maps as input in order to work: a costmap describing where the robot is allowed to and not allowed to plan paths, and a global map describing the current information the robot has about the environment.
For the cost map, we need to subscribe both to the costmap itself and any possible updates to it, as described in the following.

- `costmap` - a nav_msgs::OccupancyGrid type of costmap describing where the robot is allowed to plan paths to. All values less than or equal to 98 are interpreted as allowed for planning. For controlling behaviour in unknown areas, see the parameter `allow_unknown_targets`.
- `costmap_updates` - a map_msgs::OccupancyGridUpdates message containing updates to partial areas in the global costmap. 
- `map` a nav_msgs::OccupancyGrid type of map message containing a global map. This can be produced e.g. by a SLAM algorithm. This map is used at each planning stage as starting information, and should be updated to reflect exploration progress.


## 5. Published topics
- `planner_paths` a nav_msgs::Path type message where all paths the planner has considered on each iteration are published. Note that to visualize these e.g. in rviz, you should set the number of paths displayed equal to the number of particles defined by the parameter `num_particles`.

## 6. Parameters
These parameters can be set at node startup.
The planner behaviour control parameters set the thresholds for when the planner will think that the robot is stuck or that local trajectories are not informative.
In case these thresholds are triggered, the planner will fall back to a frontier exploration for selecting the next target and then resume planning via local trajectories.

### Frame ids
- `map_frame_id` the map frame in which planning will be done. This must match the incoming "map", "costmap", and "costmap_updates" topics' frame_id. (string, default: map)
- `base_frame_id` base frame of the robot. (string, default: base_link)

A TF transform must be available between `map_frame_id` and `base_frame_id`.

### Planner behaviour control
- `goal_execution_time` the maximum amount of time in seconds that a single exploration target will be active. Once the robot reaches the target or this limit is exceeded, it will trigger a new planning phase. (double, default: 10.0)
- `min_traj_length` minimum length of trajectories in meters for the planner to consider them valid (double, default: 0.2)
- `min_reward` minimum amount of reward units obtained for trajectories for the planner to consider them valid. Roughly speaking, this value corresponds to the the required expected number of unknown cells in the map that will be observed as either free or occupied if the robot were to traverse the trajectory. (double, default: 30.0)
- `max_sampling_tries` maximum number of samples to try to draw for each trajectory before giving up and declaring that no valid trajectory could be found. (int, default: 30)
- `frontier_distance_threshold` if the exploration target was obtained via frontier exploration, the minimum distance in meters when the robot is considered to have reached this target and planning via local trajectories is resumed. (double, default: 2.0)
- `wait_between_planner_iterations` controls whether to wait for user input (keypress) between iterations of the planning algorithm. Useful for debugging and inspecting the evolution of the trajectories considered by the planner during each iteration. (bool, default: False)

### Robot dynamics model limits
Only trajectories respecting these limits will be sampled by the planner.

- `lin_v_min` absolute minimum linear velocity for the robot in meters per second (double, default: 0.3)
- `lin_v_max` absolute maximum linear velocity for the robot in meters per second (double, default: 1.0)
- `ang_v_min` absolute minimum rotational velocity for the robot in radians per second (double, default: -1.0)
- `ang_v_max` absolute maximum rotational velocity for the robot in radians per second (double, default: 1.0)
- `f_rot_min` minimum final rotation at the end of the trajectory of the robot in radians (double, default: -0.02)
- `f_rot_max` maximum final rotation at the end of the trajectory of the robot in radians (double, default: 0.02)

## 7. Parameters (dynamically reconfigurable)
From the following parameters, the most important ones to set to correspond to your system are those controlling the simulated laser range finder.
The parameters listed below can also be reconfigured dynamically during runtime using [dynamic_reconfigure](http://wiki.ros.org/dynamic_reconfigure).

### Planning algorithm settings
- `horizon` determines how many control actions each trajectory will be composed of. (int, default: 3)
- `discount` determines the relative value of immediate and later rewards, should be greater than 0.0. The smaller this value is, the more emphasis is on immediate information gain. Value 1.0 corresponds to no preference between immediate and later information gain. (double, default: 1.0)
- `schedule_a` at iteration `i`, the planner will use `k = a*i + b` samples to evaluate each trajectory. This sets the parameter a. (int, default: 4)
- `schedule_b` at iteration `i`, the planner will use `k = a*i + b` samples to evaluate each trajectory. This sets the parameter b. (int, default: 3)
- `num_kernels` specifies the total number of iterations the planner will run for. At the first iteration, trajectories are sampled from a uniform distribution, and at later iterations from Gaussian kernels centred at the previous evaluated trajectories. (int, default: 5) 
- `std_vel` (double, default: 0.2), `std_ang` (double, default: 0.1), `std_fr` (double, default: 0.02). These parameters control the trajectory sampling process by defining the standard deviations of the Gaussian kernels for the linear velocity (`std_vel`), angular velocity (`std_ang`) and a final rotation at the end of the trajectory (`std_fr`). Velocity is measured in meters per second, angles in radians. At each iteration, the Gaussian kernel used to modify each trajectory samples from three independent Gaussian distributions. The standard deviations for these kernels decrease as function of the iteration. At iteration `i`, the standard deviation applied will be `std_vel  / (i+1)`, and so on. This is necessary so the planner does not "lose" good trajectories by modifying them too much at later iterations.
- `num_particles` the number of trajectories maintained over the set of iterations. The larger the number, the more likely it is to converge to a good trajectory. Increasing this parameter will also increase the computational demands the most. (int, default: 10)
- `resample_thresh` the threshold (between 0.0 and 1.0) for the effective number of particles below which resampling will be triggered. (double, default: 0.33)


### Planning settings and constraints
- `allow_unknown_targets` controls whether the planner is allowed to produce exploration targets that are not in free space. Setting this false will make exploration more conservative, as only areas known to be free will be set as exploration goals. (bool, default: True)
- `default_ctrl_duration` the duration of each control action in seconds. Multiplied by `horizon`, this determines the duration of the trajectories considered by the planner. (double, default: 1.0)

### Simulated laser scanner parameters
The node samples observations via raytracing with a simulated laser range finder assuming a map hypothesis consistent with the current costmaps. These observation samples are applied to  evaluate the trajectories. The simulated laser parameters can be controlled through these parameters.

- `laser_min_angle_deg` the smallest incidence angle of the laser beam in degrees. (double, default: -90)
- `laser_max_angle_deg` the largest incidence angle of the laser beam in degrees. (double, default: 90)
- `laser_angle_step_deg` the step in degrees between each laser ray. For example, using the default parameters will produce a ray in incidence angles -90, -85, ..., 85, 90. (double, default: 5.0)
- `laser_max_dist_m` the maximum measurable distance for the laser range finder in meters. (double, default: 4.0)
- `laser_p_false_pos` probability of false positive reading for the laser (free cell observed as occupied) (double, default: 0.05)
- `laser_p_false_neg` probability of false negative reading for the laser (occupied cell observed as free)(double, default: 0.05)


## 8. Tips on improving computational performance
The planning method is computationally intensive.
To improve computation speed, you can primarily try to to decrease the length of trajectories by lowering `horizon`, decreasing the number of particles `num_particles`, or the number of samples used in evaluation via `schedule_a` and `schedule_b`, or by decreasing the resolution of the simulated laser by lowering the maximum range `laser_max_dist_m` or making the resolution `laser_angle_step` larger.

## 9. Limitations
The software has been written assuming a robot in a planar environment, specifically a Turtlebot-like robot platform.
Possible robot trajectories are sampled under this assumption.
If your robot is not even barely correspond to these assumptions, the dynamics model should be replaced by one suitable for your robot.

The software currently works on 2D occupancy grid maps and costmaps.
Although there are no plans to extend the software beyond this I have attempted to write the code to enable extensions.

