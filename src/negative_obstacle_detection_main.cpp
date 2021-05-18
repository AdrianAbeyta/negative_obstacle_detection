///////////////////////////////////////////////////////////////////////////////
//      Title     : negative_obstacle_detection_main.cpp
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

#include "negative_obstacle_detection/PointCloudToLaser.h"
#include "negative_obstacle_detection/FilterPCL.h"
#include <ros/ros.h> 
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/transforms.h>
// Forward declare settings struct
struct Params;

// Forward declare LoadSettings functions
void LoadParams(Params paramaters);

struct Params{

    float leaf_size_,floor_projection_min_height_,floor_projection_max_height_,
          neg_lim_filter_min_height_,neg_lim_filter_max_height_,laser_max_height_,
          laser_min_height_,virtual_floor_max_height_,virtual_floor_min_height_,
          point_to_laser_max_height_,point_to_laser_min_height_;

    double y_prime_;  
};

void LoadParams(Params& params, ros::NodeHandle nh){
    
    // Load parameter
    if (!nh.getParam("negative_obstacle_detection/leaf_size",params.leaf_size_)){

        ROS_ERROR_STREAM("Parameter negative_obstacle_detection must be set but could not be loaded");
        throw std::invalid_argument("Could not load parameter: negative_obstacle_detection");
    }
    

    nh.param<float>("negative_obstacle_detection/leaf_size", params.leaf_size_, 0.06f );
    nh.param<float>("negative_obstacle_detection/floor_projection_min_height", params.floor_projection_min_height_, 0.30 );
    nh.param<float>("negative_obstacle_detection/floor_projection_max_height", params.floor_projection_max_height_, 0.40 );
    nh.param<float>("negative_obstacle_detection/neg_lim_filter_min_height", params.neg_lim_filter_min_height_, 0.20 );
    nh.param<float>("negative_obstacle_detection/neg_lim_filter_max_height", params.neg_lim_filter_max_height_, 5.0 );
    nh.param<float>("negative_obstacle_detection/laser_max_height", params.laser_max_height_, 0.35 );
    nh.param<float>("negative_obstacle_detection/laser_min_height", params.laser_min_height_, -0.25 );  
    nh.param<float>("negative_obstacle_detection/virtual_floor_max_height", params.virtual_floor_max_height_, 5.0 );
    nh.param<float>("negative_obstacle_detection/virtual_floor_min_height", params.virtual_floor_min_height_, 0.9 );
    nh.param<double>("negative_obstacle_detection/y_prime", params.y_prime_, 0.25 );
    nh.param<float>("negative_obstacle_detection/point_to_laser_max_height", params.point_to_laser_max_height_, 0.35 );
    nh.param<float>("negative_obstacle_detection/point_to_laser_min_height", params.point_to_laser_min_height_, -0.25 );

    return;
}

// Paramaters variable. 
Params params;

// Final Laserscans 
sensor_msgs::LaserScan  A;
sensor_msgs::LaserScan  B;
sensor_msgs::LaserScan  C;
sensor_msgs::LaserScan  combined_scans;

// Pointcloud manipulation 
pcl::PointCloud<pcl::PointXYZ>::Ptr negative_cloud_filter ( new pcl::PointCloud<pcl::PointXYZ> );
pcl::PointCloud<pcl::PointXYZ>::Ptr floor_cloud_filter ( new pcl::PointCloud<pcl::PointXYZ> );


// Pointcloud type conversions 
pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud2_to_pcl (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr camera_cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

void pointcloudCallback( const sensor_msgs::PointCloud2ConstPtr& camera_cloud, const sensor_msgs::PointCloud2ConstPtr& lidar_cloud){
    
    //////////// NEGATIVE OBSTACLE DETECTION ////////////
    // Convert camera sensor_msg to pcl
    pointcloud2_to_pcl = PointCloud2_To_PCL(camera_cloud);
    
    // Filter the raw camrea points 
    Filter_Camera_Points(params.leaf_size_, pointcloud2_to_pcl, camera_cloud_filtered);
    
    // Filter the camrea points for desired useage (Furthest Point or Virtual Floor)
    FloorProjection( params.floor_projection_min_height_, params.floor_projection_max_height_, camera_cloud_filtered, floor_cloud_filter );
    NegativeLimitFilter( params.neg_lim_filter_min_height_, params.neg_lim_filter_max_height_, camera_cloud_filtered , negative_cloud_filter );
    
    //Convert camera points back to sensor msgs pointcloud 2 msg;
    sensor_msgs::PointCloud2  final_camera_points_B;
    pcl::toROSMsg( *negative_cloud_filter, final_camera_points_B);

    //Convert camera points back to sensor msgs pointcloud 2 msg;
    sensor_msgs::PointCloud2  final_camera_points_A;
    pcl::toROSMsg( *floor_cloud_filter, final_camera_points_A);
    
    // Get resulting laserscan msg for each method 
    A = FurthestPointExtraction( final_camera_points_A );
    B = VirtualFloorProjection( params.virtual_floor_min_height_, params.virtual_floor_max_height_, params.y_prime_, final_camera_points_B );
    
    //////////// OBSTACLE DETECTION ////////////

    // Get resdulting laserscan from filtered point clouds. 
    C = PointToLaser( params.point_to_laser_min_height_, params.point_to_laser_max_height_,lidar_cloud);
    
    // Combine the respective methods Laserscan into a single scan. 
    combined_scans = CombineLaserScans( B, C, combined_scans);
}

int main(int argc, char** argv) {
    
    ros::init(argc, argv, "object_detection_node");
    
    ros::NodeHandle nh;
    
    LoadParams(params, nh);

    message_filters::Subscriber<sensor_msgs::PointCloud2> camera_cloud_sub(nh,"/camera/depth/color/points", 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_cloud_sub(nh,"/velodyne_points", 1);
   
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2,sensor_msgs::PointCloud2> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    
    boost::shared_ptr<Sync> S;
    S.reset(new Sync( MySyncPolicy(5), camera_cloud_sub, lidar_cloud_sub));
    S->registerCallback( boost::bind(&pointcloudCallback, _1, _2) );

    ros::Publisher velodyne_laser_pub = nh.advertise<sensor_msgs::LaserScan>("/scan_multi", 1);

    ros::Rate loop_rate(40); 
    

    int count = 0;
    while (ros::ok())
    {   
        // Pub combined laserscan
        velodyne_laser_pub.publish(combined_scans);

        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    return 0;
}
