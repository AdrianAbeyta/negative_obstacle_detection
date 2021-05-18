///////////////////////////////////////////////////////////////////////////////
//      Title     : FilterPCL.cpp
//      Project   : negative_obstacle_detection
//      Created   : 01/15/21
//      Author    : Adrian Abeyta
//      Platforms : Ubuntu 64-bit
//      Copyright : Copyright© The University of Texas at Austin, 2021. All
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


#include "negative_obstacle_detection/FilterPCL.h"
#include <iostream>  
#include <numeric>
#include <nodelet/nodelet.h>
#include <algorithm>
#include <vector>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include "geometry_msgs/Point32.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"

#include "ros/ros.h"
#include "ros/ros_helpers.h"
#include "math/math_util.h"
#include "nav_msgs/OccupancyGrid.h"

#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/filters/crop_box.h>

   
using geometry::line2f;
using std::cout;
using std::endl;
using std::string;
using std::swap;
using std::vector;
using Eigen::Vector2f;
using nav_msgs::OccupancyGrid;


void Filter_Camera_Points( float leaf_size, const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,  pcl::PointCloud<pcl::PointXYZ>::Ptr camera_cloud_filtered)

{
    //Create the filtering object for camera 
    pcl::VoxelGrid<pcl::PointXYZ> sor_cam;
    sor_cam.setInputCloud (cloud);

    sor_cam.setLeafSize (leaf_size , leaf_size, leaf_size);
    sor_cam.filter (*camera_cloud_filtered);

};

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloud2_To_PCL( const sensor_msgs::PointCloud2ConstPtr& cloud)

{
    //Convert From sensor_msgs::PointCloud2::ConstPtr to a pcl::PointCloud<pcl::pointxyz>::Ptr
    pcl::PCLPointCloud2 pcl_pc2_2;
    pcl_conversions::toPCL(*cloud, pcl_pc2_2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_camera_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2_2,*temp_camera_cloud);
    
    return temp_camera_cloud;
};

//// Camera Method /////

void FloorProjection( float floor_projection_min_height_,float floor_projection_max_height_,const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered ){
    
    // Create the filtering object
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud);

    // Camera's coordnate system is different from LIDAR, Y is vertical with positive pointing down. 
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (floor_projection_min_height_, floor_projection_max_height_); 
    
    // //Fill in the new filter
    pass.filter (*cloud_filtered);
    
};

void NegativeLimitFilter(float neg_lim_filter_min_height_, float neg_lim_filter_max_height_, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered ){
    
    // Create the filtering object
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud);
  
    // Camera's coordnate system is different from LIDAR, Y is vertical with positive pointing down. 
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (neg_lim_filter_min_height_, neg_lim_filter_max_height_);
   
    //Fill in the new filter
    pass.filter (*cloud_filtered);
    
};





