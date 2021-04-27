#include <ros/ros.h> 
#include <vector>
#include <list>
#include <sensor_msgs/PointCloud.h>
#include "eigen3/Eigen/Dense"
#include "../shared/math/geometry.h"
#include "../shared/math/line2d.h"
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

namespace NegativeObstacle{

    class FilterPCL{
     
      public:
      
      // Filter Raw Camera Points 
      pcl::PointCloud<pcl::PointXYZ>::Ptr Filter_Camera_Points( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr camera_cloud_filtered );

      //Convert From sensor_msgs::PointCloud2::ConstPtr to a pcl::PointCloud<pcl::pointxyz>::Ptr
      pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloud2_To_PCL( const sensor_msgs::PointCloud2ConstPtr& cloud);

      pcl::PointCloud<pcl::PointXYZ>::Ptr NegativeLimitFilter( const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered );

      pcl::PointCloud<pcl::PointXYZ>::Ptr FloorProjection( const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered );

      pcl::PointCloud<pcl::PointXYZ>::Ptr PointsToFloor( const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered );


      private:

      // Filtering Limit Paramaters
      float floor_projection_min_height_ = 0.30;  // 0.24 - SIM   // 0.30 - REAL LIFE
      float floor_projection_max_height_ = 0.40; //  0.29 - SIM  // 0.40 - REAL LIFE
      //float neg_lim_filter_min_height_ = 0.30;  // 0.3 - SIM    // 0.9 - REAL LIFE
      //float neg_lim_filter_max_height_ = 5;    //  5 - SIM     // 5 - REAL LIFE

      // Camrea Filtering Down Paramater 
      //float leaf_size_ = 0.06f;

      

    };

    class FarthestPoint{
      
      public:
        // sensor_msgs::LaserScan FillLaserMsg( sensor_msgs::LaserScan  array , pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered);
        sensor_msgs::LaserScan FurthestPointExtraction(sensor_msgs::PointCloud2 lidar_cloud);
        //sensor_msgs::LaserScan VirtualFloorProjection(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered);
        sensor_msgs::LaserScan VirtualFloorProjection(sensor_msgs::PointCloud2 lidar_cloud);
        //sensor_msgs::LaserScan PointToLaser( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered);
        sensor_msgs::LaserScan PointToLaser(const sensor_msgs::PointCloud2ConstPtr& cloud_filtered );
        
        sensor_msgs::LaserScan CombineLaserScans(sensor_msgs::LaserScan  array_a, sensor_msgs::LaserScan  array_b , sensor_msgs::LaserScan  array_c);
  
        // sensor_msgs::LaserScan RampDetection( sensor_msgs::LaserScan  array , pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered);
        void NegativeProjection( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

      private:

      // Virtual Floor Projection Paramaters 
      //float virtual_floor_max_height_ = 5;
      //float virtual_floor_min_height_ = 0.9; //sim 0.3
      //Set vertical height from laser
      //double y_prime_ = 0.25;  // Real life 0.25

    };

}
