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

#include "negative_obstacle_detection.h"
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

using NegativeObstacle::FilterPCL;
FilterPCL* point_cloud_;

using NegativeObstacle::FarthestPoint;
FarthestPoint* furthest_point_;

// Final Laserscans 
sensor_msgs::LaserScan  A_;
sensor_msgs::LaserScan  B_;
sensor_msgs::LaserScan  C_;
sensor_msgs::LaserScan  D_;

// Pointcloud manipulation 
pcl::PointCloud<pcl::PointXYZ>::Ptr negative_cloud_filter ( new pcl::PointCloud<pcl::PointXYZ> );
pcl::PointCloud<pcl::PointXYZ>::Ptr floor_cloud_filter ( new pcl::PointCloud<pcl::PointXYZ> );
pcl::PointCloud<pcl::PointXYZ>::Ptr Points_To_Floor_filter ( new pcl::PointCloud<pcl::PointXYZ> );

// Pointcloud type conversions 
pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud2_to_pcl (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr camera_cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
//pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& camera_cloud, const sensor_msgs::PointCloud2ConstPtr& lidar_cloud){
    
    //////////// NEGATIVE OBSTACLE DETECTION ////////////

    // Convert camera sensor_msg to pcl
    pointcloud2_to_pcl = point_cloud_->PointCloud2_To_PCL(camera_cloud);
    // Filter the raw camrea points 
    camera_cloud_filtered = point_cloud_->Filter_Camera_Points(pointcloud2_to_pcl,camera_cloud_filtered);
    
    // Filter the camrea points for desired useage (Furthest Point or Virtual Floor)
    //floor_cloud_filter = point_cloud_->FloorProjection( camera_cloud_filtered, floor_cloud_filter );
    negative_cloud_filter = point_cloud_->NegativeLimitFilter( camera_cloud_filtered , negative_cloud_filter );
    
    //Convert camera points back to sensor msgs pointcloud 2 msg;
    sensor_msgs::PointCloud2  final_camera_points;
    pcl::toROSMsg( *negative_cloud_filter, final_camera_points);


    // Get resulting laserscan msg for each method 
    //A_ = furthest_point_->FurthestPointExtraction( floor_cloud_filter );
    B_ = furthest_point_->VirtualFloorProjection( final_camera_points  );
    
    //////////// OBSTACLE DETECTION ////////////

    /////Points_To_Floor_filter = point_cloud_->PointsToFloor(lidar_cloud, Points_To_Floor_filter ); // TODO: SET UP A WAY TO FILTER DOWN POINTCLOUDS
    
    // Get resdulting laserscan from filtered point clouds. 
    C_ = furthest_point_->PointToLaser(lidar_cloud);
    

    // Combine the respective methods Laserscan into a single scan. 
    D_ = furthest_point_->CombineLaserScans( B_ , C_ , D_);
    
}

int main(int argc, char** argv) {
    
    ros::init(argc, argv, "object_detection_node");
    
    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::PointCloud2> camera_cloud_sub(nh,"/camera/depth/color/points", 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_cloud_sub(nh,"/velodyne_points", 1);
   
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2,sensor_msgs::PointCloud2> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    
    boost::shared_ptr<Sync> S;
    S.reset(new Sync( MySyncPolicy(5), camera_cloud_sub, lidar_cloud_sub));
    S->registerCallback( boost::bind(&pointcloudCallback, _1, _2) );

    //ros::Publisher camera_laser_pub = nh.advertise<sensor_msgs::LaserScan>("/camera_scan", 1);
    //ros::Publisher camera_filter_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("/camera_points_filter", 1000);
    
    ros::Publisher velodyne_laser_pub = nh.advertise<sensor_msgs::LaserScan>("/scan_multi", 1);
    //ros::Publisher velodyne_filter_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("/velodyne_points_filter", 1000);

    ros::Rate loop_rate(10); //10 Hz
    

    int count = 0;
    while (ros::ok())
    {   
        // Vizualize filtered pointcloud points 
        //velodyne_filter_pub.publish(*Points_To_Floor_filter);
        //camera_filter_pub.publish(*negative_cloud_filter);
        
        // Pub combined laserscan
        velodyne_laser_pub.publish(D_);

        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    return 0;
}
