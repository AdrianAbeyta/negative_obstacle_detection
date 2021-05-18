///////////////////////////////////////////////////////////////////////////////
//      Title     : FilterPCL.h
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
     
// Convert From sensor_msgs::PointCloud2::ConstPtr to a pcl::PointCloud<pcl::pointxyz>::Ptr
pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloud2_To_PCL( const sensor_msgs::PointCloud2ConstPtr& cloud);

// Filter Raw Camera Points 
void Filter_Camera_Points( float leaf_size, const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,  pcl::PointCloud<pcl::PointXYZ>::Ptr camera_cloud_filtered );
        
// Extracts pointcloud points that are below the floor plane. (CAMERA)
void NegativeLimitFilter(float neg_lim_filter_min_height_, float neg_lim_filter_max_height_ , const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered );
      
// Extracts pointcloud points that are on the floor plane. (CAMERA)
void FloorProjection(float floor_projection_min_height_,float floor_projection_max_height_, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered );
      
