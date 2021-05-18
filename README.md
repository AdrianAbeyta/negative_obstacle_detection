# negative_obstacle_detection
ROS Package for identifying edge-case obstacles using the NRG Magni robot platform.

### Runing NOD. 

The implementation of negative obstacle detection was developed and tested on ROS Melodic. Execution on other ROS versions have not been tested and my not work as intended. Note that these instructions assume that the "desktop-full" version of ROS (which includes the Gazebo physics simulator) is installed. If a the "desktop" or "base" version are instead installed, additional effort may be needed to install RVIZ.

### In separate terminal windows launch the following

1. Bringup RVIZ on the Magni robot:

```
roslaunch magni_viz view_demo.launch 
```
2. Run the negative_obstacle_detection node. 

```
roslaunch negative_obstacle_detection negative_obstacle_detection.launch
```
3. Run the slam_gmapping node. 

```
rosrun gmapping slam_gmapping scan:=scan_multi
```
### Negative Obstacle Detection Node
1. Subscribed Topics:
- camera_cloud (sensor_msgs::PointCloud2) : 3D pointcloud from a depth camera sensor. 
- lidar_cloud (sensor_msgs::PointCloud2) : 3D pointcloud from LIDAR sensor.  

2. Published Topics:
- laser (sensor_msgs::LaserScan) : Final laser scan message. 
