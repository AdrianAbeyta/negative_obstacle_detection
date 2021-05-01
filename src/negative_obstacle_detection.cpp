///////////////////////////////////////////////////////////////////////////////
//      Title     : negative_obstacle_detection.cpp
//      Project   : fast_frontier_detection
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

#include "negative_obstacle_detection.h"
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
#include "../shared/ros/ros_helpers.h"
#include "../shared/math/math_util.h"
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

#include <algorithm>   
using geometry::line2f;
using std::cout;
using std::endl;
using std::string;
using std::swap;
using std::vector;
using Eigen::Vector2f;
using nav_msgs::OccupancyGrid;
using namespace math_util;
using namespace ros_helpers;

namespace NegativeObstacle{

// //// Lidar Method /////
// pcl::PointCloud<pcl::PointXYZ>::Ptr FilterPCL::PointsToFloor( const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered ){

//     // Create the filtering object
//     pcl::PassThrough<pcl::PointXYZ> pass;
//     pass.setInputCloud (cloud);
//     pass.setFilterFieldName ("z"); //z-lidar
//     pass.setFilterLimits (-0.3, 0.35);   // camera (-0.35, 0.3) (-5, -0.5)-Lidar sim //-0.25, 0.35-LIDAR
    
//     //Fill in the new filter
//     pass.filter (*cloud_filtered);
    
//     for (auto& point : *cloud_filtered)
//     {
//         point.z = 0.0; //Camera Frame
//     }

//     return cloud_filtered;

// }

//// Camera Method /////

pcl::PointCloud<pcl::PointXYZ>::Ptr FilterPCL::FloorProjection( const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered ){

    // Create the filtering object
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud);
    
    // Camera's coordnate system is different from LIDAR, Y is vertical with positive pointing down. 
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (floor_projection_min_height_, floor_projection_max_height_); 
    
    // //Fill in the new filter
    pass.filter (*cloud_filtered);
    
    return cloud_filtered;
};

pcl::PointCloud<pcl::PointXYZ>::Ptr FilterPCL::NegativeLimitFilter( const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered ){
    
    // Create the filtering object
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud);
    
    // Camera's coordnate system is different from LIDAR, Y is vertical with positive pointing down. 
    pass.setFilterFieldName ("y");
    pass.setFilterLimits ( 0.03, 5 ); 
   
    //Fill in the new filter
    pass.filter (*cloud_filtered);
    
    return cloud_filtered;
};


pcl::PointCloud<pcl::PointXYZ>::Ptr FilterPCL::PointCloud2_To_PCL( const sensor_msgs::PointCloud2ConstPtr& cloud)

{
    //Convert From sensor_msgs::PointCloud2::ConstPtr to a pcl::PointCloud<pcl::pointxyz>::Ptr
    pcl::PCLPointCloud2 pcl_pc2_2;
    pcl_conversions::toPCL(*cloud, pcl_pc2_2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_camera_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2_2,*temp_camera_cloud);
    
    return temp_camera_cloud;
};

pcl::PointCloud<pcl::PointXYZ>::Ptr FilterPCL::Filter_Camera_Points( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr camera_cloud_filtered)

{
    //Create the filtering object for camera 
    pcl::VoxelGrid<pcl::PointXYZ> sor_cam;
    sor_cam.setInputCloud (cloud);
    sor_cam.setLeafSize (0.06f , 0.06f, 0.06f);
    sor_cam.filter (*camera_cloud_filtered);

    return camera_cloud_filtered; 
};


//////////////////////// FOR Depth Camera //////////////////////////

sensor_msgs::LaserScan FarthestPoint::FurthestPointExtraction( sensor_msgs::PointCloud2 cloud_filtered){

    sensor_msgs::LaserScan  scan;

    ros::Time scan_time = ros::Time::now();
    
    if (scan.header.frame_id.empty())
    {
      scan.header.frame_id = "camera_color_frame";
      scan.header.seq = cloud_filtered.header.seq;
    }
    //cout<< scan.header.frame_id<<endl;
    scan.header.stamp = scan_time;
    scan.angle_min = -M_PI/2;
    scan.angle_max = M_PI/2;
    scan.angle_increment = M_PI/180.0/2.0; 
    scan.time_increment = 0; 
    scan.range_min = 0.0; 
    scan.range_max = 10;
    scan.scan_time= 1.0/30.0;

    //Determine amount of rays to create
    uint32_t ranges_size = (scan.angle_max - scan.angle_min) / scan.angle_increment;
    scan.ranges.assign(ranges_size, scan.range_max + 1);

    for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud_filtered, "x"),
                                                      iter_y(cloud_filtered, "y"), 
                                                      iter_z(cloud_filtered, "z");
                                                      iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
    {
       
        if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z)) 
       {
           continue;
       }
 
       // Convert from cartesian point to polar coordnates. 
        float range = sqrt( *iter_z * *iter_z + *iter_x * *iter_x );
        float angle = atan2(*iter_x ,*iter_z );
        int index = (angle - scan.angle_min) / scan.angle_increment; 

        // Overwrite range at laserscan ray if new range is smaller
        if (scan.ranges[index] > range)
        {
            scan.ranges[index] = range;
        }

    }

    return scan; 
};

sensor_msgs::LaserScan FarthestPoint::VirtualFloorProjection( sensor_msgs::PointCloud2 cloud_filtered ){
    sensor_msgs::LaserScan  scan;

    ros::Time scan_time = ros::Time::now();
    
    if (scan.header.frame_id.empty())
    {
      scan.header.frame_id = "camera_color_frame";
      scan.header.seq = cloud_filtered.header.seq;
    }

    scan.header.stamp = scan_time;
    scan.angle_min = -M_PI;
    scan.angle_max = M_PI;
    scan.angle_increment = M_PI/360; 
    scan.time_increment = 0; 
    scan.range_min = 0; 
    //scan.range_max = std::numeric_limits<double>::max();
    scan.range_max = 200;
   
    //Determine amount of rays to create
    uint32_t ranges_size = (scan.angle_max - scan.angle_min) / scan.angle_increment;
    scan.ranges.assign(ranges_size, scan.range_max + 1);

    for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud_filtered, "x"),
                                                      iter_y(cloud_filtered, "y"), 
                                                      iter_z(cloud_filtered, "z");
                                                      iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
    {

       if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z)) 
       {
           continue;
       }
        float virtual_floor_max_height_ = 5;
        float virtual_floor_min_height_ = 0.9;
        
       if (*iter_y >virtual_floor_max_height_ || *iter_y < virtual_floor_min_height_) 
       {
           continue;
       } 
        double y_prime_ = 0.25;
        //Calulate new point coordnate.
        double z_prime = (*iter_z * y_prime_) / *iter_y;
        double x_prime = (*iter_x * z_prime)/ *iter_z;
        
        // Convert from cartesian point to polar coordnates. 
        //float range = hypot(z_prime, x_prime);
        float range = sqrt(z_prime*z_prime + x_prime*x_prime);
        if (range < scan.range_min) 
        {
            continue;
        }
        
        float angle = atan2(*iter_x,*iter_z);

         if (angle < scan.angle_min || angle > scan.angle_max) 
        {
            continue;
        }

        int index = (angle - scan.angle_min) / scan.angle_increment; 

        // Overwrite range at laserscan ray if new range is smaller
        if (scan.ranges[index] > range )
        {
            scan.ranges[index] = range;
        }
       
  }

    return scan;
};


/////////////////////// FOR 3D LIDAR //////////////////////////

sensor_msgs::LaserScan FarthestPoint::PointToLaser( const sensor_msgs::PointCloud2ConstPtr&  cloud_filtered ){

    //sensor_msgs::LaserScan  scan;
    sensor_msgs::LaserScanPtr scan(new sensor_msgs::LaserScan());
    ros::Time scan_time = ros::Time::now();
    
    //if (scan->header.frame_id.empty())
    //{
      scan->header.frame_id ="velodyne";
      scan->header.seq = cloud_filtered->header.seq;
    //}

    scan->header.stamp = scan_time;
    scan->angle_min = -M_PI;
    scan->angle_max = M_PI;
    scan->angle_increment = M_PI/360; 
    scan->time_increment = 0; 
    scan->range_min = 0.4; 
    scan->range_max = 200;
    
    // //Determine amount of rays to create
    // uint32_t ranges_size = (scan->angle_max - scan->angle_min) / scan->angle_increment;
    // //scan.ranges.assign(ranges_size, scan.range_max + 1);
    // scan->ranges.assign(ranges_size, std::numeric_limits<double>::infinity());

    uint32_t ranges_size = std::ceil((scan->angle_max - scan->angle_min) / scan->angle_increment);
    scan->ranges.assign(ranges_size, scan->range_max + 1);
    
    float max_height_ = 0.35;
    float min_height_ = -0.25;

    for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_filtered, "x"),
    iter_y(*cloud_filtered, "y"), iter_z(*cloud_filtered, "z");
    iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
  {
       if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z)) 
       {
           continue;
       }
       if (*iter_z > max_height_ || *iter_z < min_height_) 
       {
           continue;
       } 

       float range = sqrt( *iter_x * *iter_x + *iter_y * *iter_y);

        if (range < scan->range_min) 
        {
            continue;
        }

        float angle = atan2(*iter_y, *iter_x)+scan->angle_increment;

        if (angle < scan->angle_min || angle > scan->angle_max) 
        {
            continue;
        }
        
        int index = (angle - scan->angle_min) / scan->angle_increment; 

        if (range < scan->ranges[index])
        {
            scan->ranges[index] = range;
        }
  }
    return std::move(*scan); 
};

sensor_msgs::LaserScan FarthestPoint::CombineLaserScans( sensor_msgs::LaserScan  array_a, sensor_msgs::LaserScan  array_b , sensor_msgs::LaserScan  array_c){
    
    // Initialized with value from array A
    array_c = array_a;

    for ( int i = 0; i < array_c.ranges.size(); ++i )
    {
        if ( array_c.ranges[i] > array_b.ranges[i]  )
            {
                array_c.ranges[i] = array_b.ranges[i];
            }
    }

    return array_c;
};

}; //End of Negative obstacle namespace.

