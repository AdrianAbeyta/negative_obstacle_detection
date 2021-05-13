# negative_obstacle_detection
ROS Package for identifying edge-case obstacles using the NRG Magni robot platform.

### Runing The Negative Obstacle Detection Node. 

The implementation of NOD was developed and tested on ROS Melodic. Execution on other ROS versions have not been tested and my not work as intended. For execution of the NOD algorithm please follow the steps below. Note that these instructions assume that the "desktop-full" version of ROS (which includes the Gazebo physics simulator) is installed. If a the "desktop" or "base" version are instead installed, additional effort may be needed to install Gazebo and maybe RVIZ. Note also that the implementation of NOD was used with the magni_robot package and the respective ROS subscribers may need to be changed for proper integration. 

### In separate terminal windows launch the following

1. Bringup RVIZ on the Magni robot:

```
roslaunch magni_viz view_demo.launch 
```
2. Run the negative_obstacle_detection node. 

```
rosrun negative_obstacle_detection negative_obstacle_deteion_node
```
3. Run the slam_gmapping node. 

```
rosrun gmapping slam_gmapping scan:=scan_multi
```
