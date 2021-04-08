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
sensor_msgs::LaserScan  A_;
sensor_msgs::LaserScan  B_;
sensor_msgs::LaserScan  C_;
sensor_msgs::LaserScan  D_;
sensor_msgs::LaserScan  E_;

pcl::PointCloud<pcl::PointXYZ>::Ptr negative_cloud_filter ( new pcl::PointCloud<pcl::PointXYZ> );
pcl::PointCloud<pcl::PointXYZ>::Ptr floor_cloud_filter ( new pcl::PointCloud<pcl::PointXYZ> );
pcl::PointCloud<pcl::PointXYZ>::Ptr Points_To_Floor_filter ( new pcl::PointCloud<pcl::PointXYZ> );
pcl::PointCloud<pcl::PointXYZ>::Ptr camera_cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& camera_cloud, const sensor_msgs::PointCloud2ConstPtr& lidar_cloud){
    
    //Convert From sensor_msgs::PointCloud2::ConstPtr to a pcl::PointCloud<pcl::pointxyz>::Ptr
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*camera_cloud,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_camera_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_camera_cloud);
    
    //Convert From sensor_msgs::PointCloud2::ConstPtr to a pcl::PointCloud<pcl::pointxyz>::Ptr
    pcl::PCLPointCloud2 pcl_pc2_2;
    pcl_conversions::toPCL(*lidar_cloud,pcl_pc2_2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_lidar_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2_2,*temp_lidar_cloud);

    // Create the filtering object for camera 
    pcl::VoxelGrid<pcl::PointXYZ> sor_camera;
    sor_camera.setInputCloud (temp_camera_cloud);
    sor_camera.setLeafSize (0.03f, 0.03f, 0.03f);
    sor_camera.filter (*camera_cloud_filtered);

    // Create the filtering object for lidar 
    pcl::VoxelGrid<pcl::PointXYZ> sor_lidar;
    sor_lidar.setInputCloud (temp_lidar_cloud);
    sor_lidar.setLeafSize (0.3f, 0.3f, 0.3f);
    sor_lidar.filter (*lidar_cloud_filtered);

    floor_cloud_filter = point_cloud_->FloorProjection( camera_cloud_filtered, floor_cloud_filter );
    negative_cloud_filter = point_cloud_->NegativeLimitFilter( camera_cloud_filtered , negative_cloud_filter );
   
    A_ = furthest_point_->FurthestPointExtraction( floor_cloud_filter );
    B_ = furthest_point_->VirtualFloorProjection( negative_cloud_filter );
    C_ = furthest_point_->CombineLaserScans( A_ , B_ , C_) ;
    cout<<C_.header.frame_id<<endl;


    Points_To_Floor_filter = point_cloud_->PointsToFloor(lidar_cloud_filtered, Points_To_Floor_filter );
    D_ = furthest_point_->PointToLaser(Points_To_Floor_filter);
    cout<<D_.header.frame_id<<endl;
    //E_ = furthest_point_->CombineLaserScans( C_ , D_ , E_) ;

}


int main(int argc, char** argv) {
    
    ros::init(argc, argv, "object_detection_node");
    
    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::PointCloud2> camera_cloud_sub(nh,"/camera/depth/color/points", 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_cloud_sub(nh,"/velodyne_points", 1);
   
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2,sensor_msgs::PointCloud2> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    //message_filters::Synchronizer<MySyncPolicy> sync( MySyncPolicy(10), camera_cloud_sub, lidar_cloud_sub);
    boost::shared_ptr<Sync> S;
    S.reset(new Sync( MySyncPolicy(10), camera_cloud_sub, lidar_cloud_sub));
    S->registerCallback( boost::bind(&pointcloudCallback, _1, _2) );

    ros::Publisher camera_laser_pub = nh.advertise<sensor_msgs::LaserScan>("/camera_scan", 500);
    ros::Publisher velodyne_laser_pub = nh.advertise<sensor_msgs::LaserScan>("/velodyne_scan", 500);
    ros::Publisher filter_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("/vertical_points_filter", 100000);


    ros::Rate loop_rate(20);
    
    int count = 0;
    while (ros::ok())
    {   
        
        filter_pub.publish(*Points_To_Floor_filter);
        camera_laser_pub.publish(C_);
        velodyne_laser_pub.publish(D_);
        
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    return 0;
}



/////////////////////////////////////////

// void pointcloudCallback(const pcl::PointCloud<pcl::PointXYZ>::Ptr& camera_cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr& lidar_cloud){
    
//     // Create the filtering object
//     pcl::VoxelGrid<pcl::PointXYZ> sor;
//     sor.setInputCloud (camera_cloud);
//     sor.setLeafSize (0.03f, 0.03f, 0.03f);
//     sor.filter (*camera_cloud_filtered);

//     floor_cloud_filter = point_cloud_->FloorProjection( camera_cloud_filtered, floor_cloud_filter );
//     negative_cloud_filter = point_cloud_->NegativeLimitFilter( camera_cloud_filtered , negative_cloud_filter );
   
//     A_ = furthest_point_->FurthestPointExtraction( floor_cloud_filter );
//     B_ = furthest_point_->VirtualFloorProjection( negative_cloud_filter );
//     //C_ = furthest_point_->CombineLaserScans( A_ , B_ , C_) ;

//     //Points_To_Floor_filter = point_cloud_->PointsToFloor(lidar_cloud,Points_To_Floor_filter );
//     //D_ = furthest_point_->PointToLaser(Points_To_Floor_filter);

//     //E_ = furthest_point_->CombineAllScans( A_ , B_ , C_, D_, E_) ;
    
// }

// void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
//     ROS_INFO("LaserScan (val,angle)=(%f,%f", msg->range_min,msg->angle_min);
// }

    // message_filters::Subscriber< pcl::PointCloud<pcl::PointXYZ>> camera_cloud_sub(nh,"/camera/depth/color/points", 1);
    // message_filters::Subscriber< pcl::PointCloud<pcl::PointXYZ>> lidar_cloud_sub(nh,"/velodyne/points", 1);
   
    // typedef message_filters::sync_policies::ApproximateTime< pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointXYZ>> MySyncPolicy;
    // typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    // //message_filters::Synchronizer<MySyncPolicy> sync( MySyncPolicy(10), camera_cloud_sub, lidar_cloud_sub);
    // boost::shared_ptr<Sync> S;
    // S.reset(new Sync( MySyncPolicy(10), camera_cloud_sub, lidar_cloud_sub));
    // S->registerCallback( boost::bind(&pointcloudCallback, _1, _2) );

    // ros::Publisher laser_pub = nh.advertise<sensor_msgs::LaserScan>("/scan", 500);
    // ros::Publisher filter_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("/vertical_points_filter", 100000);

    //ros::Subscriber sub = nh.subscribe("/scan", 1000, pointcloudCallback);
    //ros::Subscriber point_sub_camera = nh.subscribe("/camera/depth/color/points", 1000, pointcloudCallback);

//////////////////////////////////////////// 
