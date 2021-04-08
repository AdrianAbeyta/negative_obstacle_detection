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

#include <algorithm>    // std::replace

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

//// Lidar /////
pcl::PointCloud<pcl::PointXYZ>::Ptr FilterPCL::PointsToFloor( const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered ){

// Create the filtering object
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (-0.15, 0.35); // (-5, -0.5) //-0.15, 0.4
    
    //Fill in the new filter
    pass.filter (*cloud_filtered);
    
    for (auto& point : *cloud_filtered)
    {
        point.z = 0.0;
    }

    return cloud_filtered;

}

//// Camera /////

pcl::PointCloud<pcl::PointXYZ>::Ptr FilterPCL::FloorProjection( const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered ){

    // Create the filtering object
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud);
    
    // pass.setFilterFieldName ("z");
    // pass.setFilterLimits (-10, -0.5); //(-0.7,-0.49)
    
            // Camera //
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (0.30, 0.40); //0.24, 0.29
    
    // //Fill in the new filter
    pass.filter (*cloud_filtered);
    


    return cloud_filtered;
};

pcl::PointCloud<pcl::PointXYZ>::Ptr FilterPCL::NegativeLimitFilter( const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered ){
    
    // Create the filtering object
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud);
    // pass.setFilterFieldName ("z");
    // pass.setFilterLimits (-10, -0.7); // (-5, -0.5)
    
            // Camera //
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (0.35, 10); // (-5, -0.5) // (higher # = down from bottowm, higher number = up from top? )
   
    //Fill in the new filter
    pass.filter (*cloud_filtered);
    
    return cloud_filtered;
};

// ///////////////////// FOR Depth Camera //////////////////////////
sensor_msgs::LaserScan FarthestPoint::FurthestPointExtraction( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered){

    Array A;
    sensor_msgs::LaserScan  scan;

    ros::Time scan_time = ros::Time::now();
    
    if (scan.header.frame_id.empty())
    {
      scan.header.frame_id = "camera_color_frame";
      scan.header.seq = cloud_filtered->header.seq;
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

    for(auto i = 0 ; i < cloud_filtered->points.size(); ++i)
    {
        double point_y = cloud_filtered->points[i].y;
        double point_x = cloud_filtered->points[i].x;
        double point_z = cloud_filtered->points[i].z;
        
           
        // Convert from cartesian point to polar coordnates. 
        double range = sqrt( point_z*point_z + point_x*point_x );
        double angle = atan2(point_x,point_z);
        int index = (angle - scan.angle_min) / scan.angle_increment; 

        // Overwrite range at laserscan ray if new range is smaller
        if (scan.ranges[index] > range)
        {
            scan.ranges[index] = range;
        }

    }

    return scan; 
};

sensor_msgs::LaserScan FarthestPoint::VirtualFloorProjection( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered ){
    
    Array A;
    sensor_msgs::LaserScan  scan;

    ros::Time scan_time = ros::Time::now();
    
    if (scan.header.frame_id.empty())
    {
      scan.header.frame_id = "camera_color_frame";
      scan.header.seq = cloud_filtered->header.seq;
    }

    scan.header.stamp = scan_time;
    scan.angle_min = -M_PI;
    scan.angle_max = M_PI;
    scan.angle_increment = M_PI/360; 
    scan.time_increment = 0; 
    scan.range_min = 0; 
    //scan.range_max = std::numeric_limits<double>::max();
    scan.range_max = 100;
    
    //Determine amount of rays to create
    uint32_t ranges_size = (scan.angle_max - scan.angle_min) / scan.angle_increment;
    scan.ranges.assign(ranges_size, scan.range_max + 1);
    
    // Set vertical height from laser
    double y_prime = 0.25;
   
    for(auto i = 0 ; i < cloud_filtered->points.size(); ++i)
    {
        double point_y = cloud_filtered->points[i].y;
        double point_x = cloud_filtered->points[i].x;
        double point_z = cloud_filtered->points[i].z;
           
        //Calulate new point coordnate.
        double z_prime = (point_z*y_prime)/point_y;
        double x_prime = (point_x*z_prime)/point_z;
        
        // Convert from cartesian point to polar coordnates. 
        double range = hypot(z_prime, x_prime);
        double angle = atan2(point_x,point_z);
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

sensor_msgs::LaserScan FarthestPoint::PointToLaser( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered){

    Array A;
    sensor_msgs::LaserScan  scan;

    ros::Time scan_time = ros::Time::now();
    
    if (scan.header.frame_id.empty())
    {
      scan.header.frame_id = cloud_filtered->header.frame_id;
      scan.header.seq = cloud_filtered->header.seq;
    }

    scan.header.stamp = scan_time;
    scan.angle_min = -M_PI;
    scan.angle_max = M_PI;
    scan.angle_increment = M_PI/360; 
    scan.time_increment = 0; 
    scan.range_min = 0.0; 
    scan.range_max = 100;
    
    //Determine amount of rays to create
    uint32_t ranges_size = (scan.angle_max - scan.angle_min) / scan.angle_increment;
    //scan.ranges.assign(ranges_size, scan.range_max + 1);
    scan.ranges.assign(ranges_size, std::numeric_limits<double>::infinity());

    for(auto i = 0 ; i < cloud_filtered->points.size(); ++i)
    {
        double point_y = cloud_filtered->points[i].y;
        double point_x = cloud_filtered->points[i].x;
        double point_z = cloud_filtered->points[i].z;
        
           
        // Convert from cartesian point to polar coordnates. 
        double range = sqrt(point_x*point_x + point_y*point_y);
        double angle = atan2(point_y, point_x);
        int index = (angle - scan.angle_min) / scan.angle_increment; 

        if (range < scan.ranges[index])
        {
            scan.ranges[index] = range;
        }
    }

    return scan; 
};

sensor_msgs::LaserScan FarthestPoint::CombineLaserScans( sensor_msgs::LaserScan  array_a, sensor_msgs::LaserScan  array_b , sensor_msgs::LaserScan  array_c){
    
    // array_c = array_a;
    // for ( int i = 0; i < array_c.ranges.size(); ++i )
    //  {
    //     array_b.ranges.push_back(array_c.ranges[i]);

    //  }

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

sensor_msgs::LaserScan FarthestPoint::CombineAllScans( sensor_msgs::LaserScan  array_a, sensor_msgs::LaserScan  array_b , sensor_msgs::LaserScan  array_c,sensor_msgs::LaserScan  array_d,sensor_msgs::LaserScan  array_e ){
    
    array_c = array_a;
    for ( int i = 0; i < array_c.ranges.size(); ++i )
     {
        array_b.ranges.push_back(array_c.ranges[i]);
     }
    
    array_e = array_b;
    for ( int i = 0; i < array_e.ranges.size(); ++i )
     {
        array_d.ranges.push_back(array_e.ranges[i]);
     }

    return array_d;
};


}; //End of Negative obstacle namespace.




// sensor_msgs::LaserScan FarthestPoint::FurthestPointExtraction( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered){

//     sensor_msgs::LaserScan  scan;

//     ros::Time scan_time = ros::Time::now();
    
//     if (scan.header.frame_id.empty())
//     {
//       scan.header.frame_id = cloud_filtered->header.frame_id;
//       scan.header.seq = cloud_filtered->header.seq;
//     }
    
//     scan.header.stamp = scan_time;
//     scan.angle_min = -M_PI;
//     scan.angle_max = M_PI;
//     scan.angle_increment = M_PI/360; 
//     scan.time_increment = 0; 
//     scan.range_min = 0.0; 
//     scan.range_max = 100;
    
//     //Determine amount of rays to create
//     uint32_t ranges_size = (scan.angle_max - scan.angle_min) / scan.angle_increment;
//     scan.ranges.assign(ranges_size, scan.range_max + 1);

//     for(auto i = 0 ; i < cloud_filtered->points.size(); ++i)
//     {
//         double point_y = cloud_filtered->points[i].y;
//         double point_x = cloud_filtered->points[i].x;
//         double point_z = cloud_filtered->points[i].z;
        
           
//         // Convert from cartesian point to polar coordnates. 
//         double range = hypot(point_x, point_y);
//         double angle = atan2(point_y, point_x);
//         int index = (angle - scan.angle_min) / scan.angle_increment; 

//         if (range < scan.ranges[index])
//         {
//             scan.ranges[index] = range;
//         }
//     }

//     // //Finding the range
//     // double min = *min_element(scan.ranges.begin(), scan.ranges.end());
//     // for(auto i = 0 ; i < cloud_filtered->points.size(); ++i)
//     // {
//     //     scan.ranges.push_back(min);
//     // }

//     return scan; 
// };

// sensor_msgs::LaserScan FarthestPoint::VirtualFloorProjection( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered ){
    
//     sensor_msgs::LaserScan  scan;

//     ros::Time scan_time = ros::Time::now();
    
//     if (scan.header.frame_id.empty())
//     {
//       scan.header.frame_id = cloud_filtered->header.frame_id;
//       scan.header.seq = cloud_filtered->header.seq;
//     }

//     scan.header.stamp = scan_time;
//     scan.angle_min = -M_PI;
//     scan.angle_max = M_PI;
//     scan.angle_increment = M_PI/360; 
//     scan.time_increment = 0; 
//     scan.range_min = 0.0; 
//     //scan.range_max = std::numeric_limits<double>::max();
//     scan.range_max = 100;
    
//     //Determine amount of rays to create
//     uint32_t ranges_size = (scan.angle_max - scan.angle_min) / scan.angle_increment;
//     scan.ranges.assign(ranges_size, scan.range_max + 1);
    
//     // Set vertical height from laser
//     double z_prime = -0.3;
   
//     for(auto i = 0 ; i < cloud_filtered->points.size(); ++i)
//     {
//         double point_y = cloud_filtered->points[i].y;
//         double point_x = cloud_filtered->points[i].x;
//         double point_z = cloud_filtered->points[i].z;
           
//         //Calulate new point coordnate.
//         double y_prime = (point_y*z_prime)/point_z;
//         double x_prime = (point_x*y_prime)/point_y;
        
        
//         // Convert from cartesian point to polar coordnates. 
//         double range = sqrt(x_prime*x_prime + y_prime*y_prime);
//         double angle = atan2(point_y, point_x);
//         int index = (angle - scan.angle_min) / scan.angle_increment; 

//         // Overwrite range at laserscan ray if new range is smaller
//         if (scan.ranges[index] > range )
//         {
//             scan.ranges[index] = range;
//         }

//     }

//     return scan;
// };

    // for(auto i = 0 ; i< cloud_filtered->points.size(); ++i)
    // {
    //     float point_y = cloud_filtered->points[i].y;
    //     float point_x = cloud_filtered->points[i].x;
    //     float point_z = cloud_filtered->points[i].z;
       
    //     //Calulate new point coordnate.
    //     float y_prime = (point_y*z_prime)/point_z;
    //     float x_prime = (point_x*y_prime)/point_y;

    //     // Convert from cartesian point to polar coordnates. 
    //     float range = hypot(x_prime, y_prime);
    //     float angle = atan2(y_prime, x_prime);
    //     int index = (angle - array.angle_min) / array.angle_increment;  

    //     std::cout <<"Index" <<index << std::endl;
    //     std::cout <<"Range" <<range<< std::endl;

    //     if ( A.index[i] == index )
    //     {
    //         if (range < A.range[i])
    //         {
    //             A.range[i]=range;
    //         }

    //     }          
    // }

