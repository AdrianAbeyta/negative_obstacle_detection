///////////////////////////////////////////////////////////////////////////////
//      Title     : PointCloudToLaser.h
//      Project   : negative_obstacle_detection
//      Created   : 01/15/21
//      Author    : Adrian Abeyta
//      Platforms : Ubuntu 64-bit
//      Copyright : Copyright© The University of Texas at Austin, 2014-2017. All
//      rights reserved.
//
//          All files within this directory are subject to the following, unless
//          an alternative license is explicitly included within the text of
//          each file.
//
//          This software and documentation constitute an unpublished work
//          and contain valuable trade secrets and proprietary information
//          belonging to the University. None of the foregoing material may be
//          copied or duplicated or disclosed without the express, written
//          permission of the University. THE UNIVERSITY EXPRESSLY DISCLAIMS ANY
//          AND ALL WARRANTIES CONCERNING THIS SOFTWARE AND DOCUMENTATION,
//          INCLUDING ANY WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
//          PARTICULAR PURPOSE, AND WARRANTIES OF PERFORMANCE, AND ANY WARRANTY
//          THAT MIGHT OTHERWISE ARISE FROM COURSE OF DEALING OR USAGE OF TRADE.
//          NO WARRANTY IS EITHER EXPRESS OR IMPLIED WITH RESPECT TO THE USE OF
//          THE SOFTWARE OR DOCUMENTATION. Under no circumstances shall the
//          University be liable for incidental, special, indirect, direct or
//          consequential damages or loss of profits, interruption of business,
//          or related expenses which may arise from use of software or
//          documentation, including but not limited to those resulting from
//          defects in software and/or documentation, or loss or inaccuracy of
//          data of any kind.
//
///////////////////////////////////////////////////////////////////////////////

#include <ros/ros.h> 
#include <vector>
#include <list>
#include <sensor_msgs/PointCloud.h>
#include "eigen3/Eigen/Dense"
#include "math/geometry.h"
#include "math/line2d.h"
#include "nav_msgs/OccupancyGrid.h"


#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"

#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <tf2_ros/buffer.h>


namespace ros {
  class NodeHandle;
}
        
// Takes the furthest points in the pointcloud and converts to laserscan msg.
sensor_msgs::LaserScan FurthestPointExtraction(const sensor_msgs::PointCloud2 lidar_cloud);
        
// Projetcs the pointcloud points into the virtual floor plane and converts to a laserscan msg. 
sensor_msgs::LaserScan VirtualFloorProjection( float virtual_floor_min_height_, float virtual_floor_max_height_, double y_prime_,const sensor_msgs::PointCloud2 lidar_cloud);
        
// Converts all points in the pointcloud to laserscan msg.
sensor_msgs::LaserScan PointToLaser( float point_to_laser_min_height_, float point_to_laser_max_height_, const sensor_msgs::PointCloud2ConstPtr& cloud_filtered );
        
// Combines the laserscan msgs into a single msg. 
sensor_msgs::LaserScan CombineLaserScans(const sensor_msgs::LaserScan  array_a, const sensor_msgs::LaserScan  array_b , sensor_msgs::LaserScan  array_c);
