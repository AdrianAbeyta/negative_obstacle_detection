# negative_obstacle_detection
ROS Package for identifying edge-case obstacles using the NRG Magni robot platform.

### NOD Node

1. Subscribed Topics:
- camera_cloud (sensor_msgs::PointCloud2) : 3D pointcloud from a depth camera sensor. 
- lidar_cloud (sensor_msgs::PointCloud2) : 3D pointcloud from LIDAR sensor.  

2. Published Topics:
- laser (sensor_msgs::LaserScan) : Final laser scan message. 

3. Paramaters 
- camera_cloud (sensor_msgs::PointCloud2) : 3D pointcloud from a depth camera sensor. 
- lidar_cloud (sensor_msgs::PointCloud2) : 3D pointcloud from LIDAR sensor.  
