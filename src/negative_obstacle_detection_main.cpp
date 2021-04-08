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

    struct Array {

      std::vector<double> range;
      std::vector<double> angle;
      std::vector<double> index;

    };

    class FilterPCL{
      public:

      pcl::PointCloud<pcl::PointXYZ>::Ptr NegativeLimitFilter( const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered );

      pcl::PointCloud<pcl::PointXYZ>::Ptr FloorProjection( const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered );

      pcl::PointCloud<pcl::PointXYZ>::Ptr PointsToFloor( const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered );

    };

    class FarthestPoint{
      public:
        // sensor_msgs::LaserScan FillLaserMsg( sensor_msgs::LaserScan  array , pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered);
        sensor_msgs::LaserScan FurthestPointExtraction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered);
        sensor_msgs::LaserScan VirtualFloorProjection(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered);
        sensor_msgs::LaserScan PointToLaser( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered);
        
        sensor_msgs::LaserScan CombineLaserScans(sensor_msgs::LaserScan  array_a, sensor_msgs::LaserScan  array_b , sensor_msgs::LaserScan  array_c);
        sensor_msgs::LaserScan CombineAllScans( sensor_msgs::LaserScan  array_a, sensor_msgs::LaserScan  array_b , sensor_msgs::LaserScan  array_c,sensor_msgs::LaserScan  array_d,sensor_msgs::LaserScan  array_e);
        // sensor_msgs::LaserScan RampDetection( sensor_msgs::LaserScan  array , pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered);
        void NegativeProjection( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    };

    
    // class VirtualFloor{
    //   public:
         
    //   private:

    // }
}
