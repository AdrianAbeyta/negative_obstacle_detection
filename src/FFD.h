#include <vector>
#include <list>
#include "sensor_msgs/PointCloud.h"
#include "eigen3/Eigen/Dense"
#include "../shared/math/geometry.h"
#include "../shared/math/line2d.h"
#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "visualization_msgs/Marker.h"

using nav_msgs::OccupancyGrid;


#ifndef CONTOURS_H
#define CONTOURS_H

namespace ros {
  class NodeHandle;
}


namespace FFD{

    // Describes a frontier
    struct frontier {
        sensor_msgs::PointCloud msg;
    };
    // Holds a vector of frontiers for processing
    struct frontier_vector{
        std::vector<frontier> frontiers;
    };


    class Contour {
      public:
        //Default Constructor
        Contour( ros::NodeHandle* n );
        //Generate a list of contour points (set resolution of line) from laser scan points
        void GenerateContour( const sensor_msgs::PointCloud& laser_coordinates );
        //Generate a vector of points sampled from line and appends to contour
        void SampleLine( const geometry::line2f line );
        //Return active area as ------- from current contour
        void UpdateActiveArea( const nav_msgs::Odometry& msg , const sensor_msgs::PointCloud& laser_coordinates,  geometry_msgs::TransformStamped robot_transform );
        //Returns contour data
        sensor_msgs::PointCloud GetContour();
        //Returns bounds for active area
        std::vector<float> GetActiveArea();
        std::vector<float> GetRobotPosition();
      private:
        
        std::vector<float> robot_pos_;
        sensor_msgs::PointCloud contour_; //Only one contour in the entire program
        const float resolution_; //m : line sampling
        std::vector<float> active_area_; // xmin, xmax, ymin, ymax
    };

    class FrontierDB {
      public:
      //TODO fix order in function
        
        // Default contructor
        FrontierDB( ros::NodeHandle* n );

        // Appends new frontiers from contour
        void ExtractNewFrontier( Contour& c, const nav_msgs::OccupancyGrid& g );
        bool IsCellFrontier( const nav_msgs::OccupancyGrid& g, const int x_cell, const int y_cell );
        bool InGrid(const nav_msgs::OccupancyGrid& g, const int x_cell, const int y_cell  ) const;

        
        void MaintainFrontiers( Contour& c , const nav_msgs::OccupancyGrid& graph);
        void ClearNewFrontier();
        bool FrontierOverlaps( const frontier new_frontier,const frontier current_frontier);
        bool FrontierIsEmpty(frontier frontier);

        void MergeFrontiers();
        bool WithinTolerance(geometry_msgs::Point32 point_a, geometry_msgs::Point32 point_b);

        // Frontier is a list of points, the robot goal is the average of the frontiers points. The closest average is the frontier average to go. 
        void UpdateClosestFrontierAverage(Contour& c);

        std::vector<float> GetCalculatedWaypoint(Contour c);
        geometry_msgs::PoseStamped PublishClosestFrontierAsNavGoal( std::vector<float> robot_pos );
        visualization_msgs::Marker PublishNavGoal( geometry_msgs::PoseStamped goal_msg );

        

      private:
        frontier_vector frontier_DB;
        frontier_vector new_frontiers;
        std::vector<std::vector<float>> frontier_goals; 
        std::vector<float> calculated_waypoint_;
    };

}
#endif