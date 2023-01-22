# Map Exploration

## ROS2 meta-package for 2D-map exploration based on frontier searching

This meta-package is for 2D-map exploration for wheeled robots based on Nav2 stack. The
exploration algorithm is based on frontier searching. For motion planning TEB local planner is used.

<img src="https://github.com/WFram/map_exploration/blob/master/docs/map_exploration.gif" width = 430 height = 240 />

## 1. Prerequisites

You need to pull [this](https://hub.docker.com/repository/docker/andrewfram/map_exploration/general) docker image. After that, use ``run.bash`` script to run a container and run a new bash session
by using ``into.bash``

```
docker pull andrewfram/map_exploration:latest
cd map_exploration
bash docker/run.bash
bash docker/into.bash
```

Inside the docker container build the packages and source the environment:
````
cd workspace
colcon build
source ~/.bashrc
````

## 2. Launching

In two bash sessions run the following commands to launch the gazebo environment and exploration algorithm. The TB3
should start exploration:

````
ros2 launch survey maze_slam.launch.py
ros2 run frontier_discoverer discoverer
````

## 3. Configuration

In the ``survey`` package there are configuration files that set parameters for planning and mapping algorithms. The
most important parameters to adjust are provided under ``controller_server`` tag in ``nav2_params_tb3.yaml``. If the
robot gets stuck you might need to change the mapping ``resolution`` in ``slam_toolbox_tb3.yaml``.