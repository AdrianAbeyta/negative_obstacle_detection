#include "FFD.h"
#include <sensor_msgs/PointCloud.h>
#include "geometry_msgs/Point32.h"
#include "ros/ros.h"
#include "../shared/math/math_util.h"
#include "../shared/ros/ros_helpers.h"
#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include <iostream>  
#include <numeric>  
#include <cmath>
#include <cfloat>
#include "visualization_msgs/Marker.h"
#include <queue>

using geometry::line2f;
using std::cout;
using std::endl;
using std::string;
using std::swap;
using std::vector;
using Eigen::Vector2f;
using nav_msgs::OccupancyGrid;
using FFD::frontier;
using FFD::frontier_vector;
using FFD::FrontierDB;
//using sensor_msgs::PointCloud;
//using geometry_msgs::TransformStamped;
using namespace math_util;
using namespace ros_helpers;

namespace FFD{
ros::Publisher contour_pub_;
ros::Publisher frontier_pub_;


Contour::Contour(ros::NodeHandle* n) :
    resolution_(0.05) 
    {
        contour_.header.frame_id = "/map";
        contour_pub_ = n->advertise<sensor_msgs::PointCloud> ("contour", 1);
        
    }

FrontierDB::FrontierDB(ros::NodeHandle* n) :
    frontier_DB(),
    new_frontiers()
    {
        frontier_pub_ = n->advertise<sensor_msgs::PointCloud> ("latest_frontiers", 1);
    }

void Contour::GenerateContour(const sensor_msgs::PointCloud& laser_coordinates){

    contour_.points.clear();

    for (int i = 0; i < laser_coordinates.points.size() - 1; ++i)
    {
        const float point1_x = laser_coordinates.points[i].x;
        const float point1_y = laser_coordinates.points[i].y;
        const float point2_x = laser_coordinates.points[i+1].x;
        const float point2_y = laser_coordinates.points[i+1].y;

        line2f segment(point1_x,point1_y,point2_x,point2_y);
        SampleLine(segment);
    }
    contour_pub_.publish(contour_);
    return;
}

void Contour::SampleLine(const line2f line){

    const float x_range = fabs(line.p1.x() - line.p0.x());
    const float y_range = fabs(line.p1.y() - line.p0.y()); 

    const float line_length = sqrt(pow(x_range,2) + pow(y_range,2));
    const float line_slope = (line.p1.y() - line.p0.y())/(line.p1.x() - line.p0.x());
    
    // X step is always positive because of the square root, if statement later accounts for it.
    const float x_step = sqrt(pow(resolution_,2)/(1 + pow(line_slope,2)));
    
    for (int i = 0; i*x_step<x_range; ++i)
    {
        geometry_msgs::Point32 point;
        
        if(line.p0.x() > line.p1.x())
        {
            point.x = line.p0.x() - i*x_step;
            point.y = line.p0.y() - i*x_step*line_slope;
        }
        else
        {
            point.x = line.p0.x() + i*x_step;
            point.y = line.p0.y() + i*x_step*line_slope;
        }

        point.z = 0.00;
        
        contour_.points.push_back(point);
    }
    return;
}

void Contour::UpdateActiveArea( const nav_msgs::Odometry& msg , const sensor_msgs::PointCloud& laser_coordinates,  geometry_msgs::TransformStamped robot_transform )
{
    // Calulate robot pose in map frame given transformation.
    float robot_x = robot_transform.transform.translation.x; 
    float robot_y = robot_transform.transform.translation.y;
    
    //Update Private Variable
    robot_pos_ = { robot_x, robot_y };

    float xmin = laser_coordinates.points[0].x;
    float xmax = laser_coordinates.points[0].x;
    float ymin = laser_coordinates.points[0].y;
    float ymax = laser_coordinates.points[0].y;

     //Find max and min laser values from robot frame.
     for ( const auto& point: laser_coordinates.points )
     {
         xmax = std::max(xmax,point.x);
         ymax = std::max(ymax,point.y);
         xmin = std::min(xmin,point.x);
         ymin = std::min(ymin,point.y);
     }

     // Clear old area add new one
     active_area_.clear();
     active_area_.push_back(xmin);
     active_area_.push_back(xmax);
     active_area_.push_back(ymin);
     active_area_.push_back(ymax);

}


sensor_msgs::PointCloud Contour::GetContour(){
    return contour_;
}
vector<float> Contour::GetActiveArea(){
    return active_area_;
}
std::vector<float> Contour::GetRobotPosition(){

   return robot_pos_;
}

//-------------------------------------------------------------------------
//----------------------------FRONTIER FUNCTIONS---------------------------
//-------------------------------------------------------------------------

void FrontierDB::ExtractNewFrontier(Contour& c, const nav_msgs::OccupancyGrid& g){
    const sensor_msgs::PointCloud contour = c.GetContour();
    bool last_pt_frontier = false;
    frontier f;
    f.msg.header.frame_id = "/map";

    for (auto& point : contour.points)
    {
        const float x = point.x;
        const float y = point.y;
        const int x_cell = (unsigned int)((x - g.info.origin.position.x) / g.info.resolution);
        const int y_cell = (unsigned int)((y - g.info.origin.position.y) / g.info.resolution);
        
        if(fabs(x)<200 || fabs(y)<200)//Remove extreme points from frontier
        {
        //Start populating a new frontier
        if (last_pt_frontier == false && IsCellFrontier(g,x_cell,y_cell))
        { 
            f.msg.points.push_back(point);
            last_pt_frontier = true;
        }
        //Continue populating a current frontier
        else if(last_pt_frontier == true && IsCellFrontier(g,x_cell,y_cell))
        {
            
            f.msg.points.push_back(point);
        }
        //Save populated frontier, clear and look for new starting point of a new frontier
        else if(last_pt_frontier == true && !IsCellFrontier(g,x_cell,y_cell) && !FrontierIsEmpty(f) )
        {
            new_frontiers.frontiers.push_back(f);
            last_pt_frontier = false;
            f.msg.points.clear();
        }
        }
    }

    // Combine all frontier pointclouds for visualization. 
    frontier viz_frontiers;
    viz_frontiers.msg.header.frame_id = "/map";
    for (auto& frontier:new_frontiers.frontiers)
    {
        for (auto& point:frontier.msg.points)
        {
            //const float point1_x = frontier.points[i].x;
            //const float point1_y = frontier.points[i].y;
            viz_frontiers.msg.points.push_back(point);
        }
    }

    frontier_pub_.publish(viz_frontiers.msg);
    viz_frontiers.msg.points.clear();
    
    return;
}

bool FrontierDB::IsCellFrontier(const nav_msgs::OccupancyGrid& g, const int x_cell, const int y_cell)
{
    const int x_upper = g.info.width ;
    const int y_upper = g.info.height;

    //Is center cell in 3x3 unknown space? -1
    if(g.data[x_cell+y_cell*g.info.width] == 0) 
    {
          // Find all surrounding cells by adding +/- 1 to col and row 
        for ( int col = x_cell-1; col <= x_cell+1; ++col)
        {
            for ( int row = y_cell-1; row <= y_cell+1; ++row)
            { 

                // If the cell given is not center and its within the grid.
                if ( !((col == x_cell) && (row == y_cell)) && InGrid(g,col, row) )
                {
                    int map_loc = col+row*g.info.width;
                    // Is any one of the surrounding cells open space? 0
                    if( col != x_cell && row != y_cell && g.data[map_loc] == -1)
                    {
                        return true;
                    }
                }
            }
        }
    } 

return false;
}

bool FrontierDB::InGrid( const nav_msgs::OccupancyGrid& g ,const int col, const int row  ) const
{

    const int rows_ = g.info.width ;
    const int cols_ = g.info.height;

    //Return false if row and col are negative
    if( row < 0 || 
        col < 0 ) 
    {
      return false;  
    }
  
    //Return false if values are greater than given grid.
    if( row >= rows_ || 
        col >= cols_ )
    {
      return false;    
    }

 return true;
}


void FrontierDB::MaintainFrontiers(Contour& c, const nav_msgs::OccupancyGrid& graph)
{
    //Get active area
    std::vector<float> active_area = c.GetActiveArea(); //Xmin,Xmax,Ymin,Ymax

    // Remove frontierDB points in the active area
    // Check if between x min and x max and if beween y min and y max
    for(unsigned int x = active_area[0]; x<=active_area[1]; ++x)
    {
       
        for(unsigned int y = active_area[2]; y<active_area[3]; ++y)
        {
            int p_x=x;
            int p_y=y;

            // Calulate cell position.
            int x_cell = (unsigned int)((p_x - graph.info.origin.position.x) / graph.info.resolution);
            int y_cell = (unsigned int)((p_y - graph.info.origin.position.y) / graph.info.resolution);
            
            if ( IsCellFrontier(graph,x_cell,y_cell) )
            {   
                // split the current frontier into two partial frontiers
                int Enables_f = -1;
                int Enables_p = -1;

                for (unsigned int i = 0; i < frontier_DB.frontiers.size(); ++i)
                {
                    for (unsigned int j = 0; j < frontier_DB.frontiers[i].msg.points.size(); ++j) 
                    {
                        
                        if( frontier_DB.frontiers[i].msg.points[j].x == p_x && frontier_DB.frontiers[i].msg.points[j].y == p_y )
                        {
                            Enables_f = i;
                            Enables_p = j;
                        }
                    }//j
                }//i
                
                if(Enables_f == -1 || Enables_p == -1)
                    continue;

                frontier f1;
                frontier f2;
                
                for(unsigned int i=0; i<=Enables_p; i++)
                {
                    f1.msg.points.push_back( frontier_DB.frontiers[Enables_f].msg.points[i] );
                }
                
                for(unsigned int i=Enables_p+1; i<frontier_DB.frontiers[Enables_f].msg.points.size(); i++)
                {
                    f2.msg.points.push_back( frontier_DB.frontiers[Enables_f].msg.points[i] );
                }
                
                frontier_DB.frontiers.erase( frontier_DB.frontiers.begin() + Enables_f );
            }//if p is a frontier
        }//for x
    }//for y
    
    //Update frontier_goal_choices pts
    frontier_goals.clear();
    float sum_x = 0.0;
    float sum_y = 0.0;
   
    for(auto& frontier:new_frontiers.frontiers)
    {
        
        for ( auto& point:frontier.msg.points)
        {  

            sum_x += point.x;
            sum_y += point.y;
        }
       
    
        float x_average = sum_x/frontier.msg.points.size();
        float y_average = sum_y/frontier.msg.points.size();

        vector<float> frontier_average_pt = {x_average,y_average};
        frontier_goals.push_back(frontier_average_pt);
        
    
    }
    
    
 ////////////////////////////////////////////////

//Storing new detected frontiers
int ActiveArea[ 1000 ][ 1000 ];
for (unsigned int i = 0; i < frontier_DB.frontiers.size(); ++i)
   {
       for (unsigned int j = 0; j < frontier_DB.frontiers[i].msg.points.size(); ++j)
       {
           int x = frontier_DB.frontiers[i].msg.points[j].x;
           int y = frontier_DB.frontiers[i].msg.points[j].y;
           ActiveArea[x][y] = i;
       }
   }

for( unsigned int i=0; i<new_frontiers.frontiers.size(); ++i)
     {
         frontier f = new_frontiers.frontiers[i];
         bool overlap = 0;

         for(unsigned int j=0; j< f.msg.points.size(); ++j)
        {
            int x = f.msg.points[j].x;
            int y = f.msg.points[j].y;
            
            if( ActiveArea[x][y] != 0 ) //overlap
            {
                int exists = ActiveArea[x][y]; 
                //merge f and exists
                for( int merged=0; merged<f.msg.points.size(); ++merged)
                {
                    frontier_DB.frontiers[exists].msg.points.push_back(f.msg.points[merged]);  
                }

                new_frontiers.frontiers[i].msg.points.clear();
                overlap = 1;
                break;
            }
        }//for j

        if ( overlap == 0)
        {
            frontier_DB.frontiers.push_back(f);
        }

     } //for i


for( unsigned int i=0; i< frontier_DB.frontiers.size(); i++)
   {
       if( frontier_DB.frontiers[i].msg.points.size() == 0 )
       {
           frontier_DB.frontiers.erase(frontier_DB.frontiers.begin() + i);
       }
   } 


//////////////////////////////////////////////////
    // Merge New Frontiers with existing frontiers. 
    //MergeFrontiers();
    
    // Clear the new frontiers db
    //ClearNewFrontier();
       
}



bool FrontierDB::FrontierIsEmpty(frontier frontier)
{   
    //std::cout << " This is size:  "<< frontier.msg.points.size() << std::endl;
    if ( frontier.msg.points.empty() )
    {
        //std::cout << "This is empty" << std::endl;
        return true;
    }

return false;
}

void FrontierDB::ClearNewFrontier()
{
    // Clear new frontier vector 
    for(auto& frontier : new_frontiers.frontiers )
    {
        frontier.msg.points.clear();
    }

    for( unsigned int i=0; i< frontier_DB.frontiers.size(); i++)
   {
       if( frontier_DB.frontiers[i].msg.points.size() == 0 )
       {
           frontier_DB.frontiers.erase(frontier_DB.frontiers.begin() + i);
       }
   }

return;
}

bool FrontierDB::FrontierOverlaps(const frontier new_frontier,const frontier current_frontier)
{
    // Return true if frontier overlaps. 
    for ( auto& point_new: new_frontier.msg.points)
    {
        for (auto& point_old: current_frontier.msg.points)
        {
            return WithinTolerance(point_new,point_old);
        }  
    }

 return false;
}

void FrontierDB::MergeFrontiers()
{
    if (new_frontiers.frontiers.size()> 0 ){
    // Itterate through old and new frontiers
    for(auto& frontier_new: new_frontiers.frontiers)
    {
        bool frontier_match_found = false;
        
        for(auto& frontier_old: frontier_DB.frontiers)
        {
            
            
            // If they overlap, merge the frontiers. 
            if( FrontierOverlaps(frontier_new,frontier_old) )
            {
                for( auto& point_new: frontier_new.msg.points)
                {
                    for(auto& point_old: frontier_old.msg.points)
                    {
                        if( ! WithinTolerance( point_new, point_old ) )
                        {
                            frontier_old.msg.points.push_back(point_new);
                            break;
                        }
                    }
                }
            
            frontier_match_found = true;  
            }      
        }

        // Add frontier to the frontiers database.  
        if(frontier_match_found == false)
        {
            frontier_DB.frontiers.push_back(frontier_new);
        }
    }
    }
return;  
}

bool FrontierDB::WithinTolerance(geometry_msgs::Point32 point_a, geometry_msgs::Point32 point_b)
{
    const float tolerance = 0.10; // m 0.05

    float x = point_a.x;
    float x1 = point_b.x;
    float y = point_a.y;
    float y1 = point_b.y;

    float length = sqrt(pow(x-x1,2) + pow(y-y1,2));
            
    // Tolerance check. 
    if(length <= tolerance)
    {
        return true;
    }

 return false; 
}

void FrontierDB::UpdateClosestFrontierAverage( Contour& c )
{

    std::vector<float> robot_pos = c.GetRobotPosition();
    float goal_distance = 300000.0;
    vector<float> goal;
    

    if (frontier_goals.size() > 0)
    {
        for ( auto& frontier_pt: frontier_goals )
        {   
            if ( !isinf(frontier_pt[0]) && !isinf(frontier_pt[1]) )
            {

                float distance_to_pt = sqrt(pow(frontier_pt[0]-robot_pos[0],2) + pow(frontier_pt[1]-robot_pos[1],2));
                
                //Get shortest distance to point. 
                if ( distance_to_pt < goal_distance ) 
                {
                    //Offset the frontier point so move_base will take as valid goal. 
                    frontier_pt[0] = frontier_pt[0]+0.1;
                    frontier_pt[1] = frontier_pt[1]+0.1;
                    
                    //set goal and update distance. 
                    goal = frontier_pt;
                    goal_distance = distance_to_pt;
                }
            }
        }
            //Set private variable goal waypoint
            calculated_waypoint_ = goal;   
    }
return;
}

geometry_msgs::PoseStamped FrontierDB::PublishClosestFrontierAsNavGoal( vector<float> robot_pos )
{

    geometry_msgs::PoseStamped goal_msg;
    goal_msg.header.frame_id = "map";
    goal_msg.header.stamp = ros::Time::now();

    goal_msg.pose.position.x = robot_pos[0]; 
    goal_msg.pose.position.y = robot_pos[1];
    goal_msg.pose.position.z = 0.0;
    
    goal_msg.pose.orientation.x = 0.0;
    goal_msg.pose.orientation.y = 0.0;
    goal_msg.pose.orientation.z = 0.0;
    goal_msg.pose.orientation.w = 1.0; 

    return goal_msg;  
}

visualization_msgs::Marker FrontierDB::PublishNavGoal( geometry_msgs::PoseStamped goal_msg )
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = goal_msg.header.frame_id;
    marker.header.stamp = goal_msg.header.stamp;
    marker.ns = "my_namespace";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = goal_msg.pose.position.x;
    marker.pose.position.y = goal_msg.pose.position.y;
    marker.pose.position.z = goal_msg.pose.position.z;
    marker.pose.orientation.x = goal_msg.pose.orientation.x;
    marker.pose.orientation.y = goal_msg.pose.orientation.y;
    marker.pose.orientation.z = goal_msg.pose.orientation.z;
    marker.pose.orientation.w = goal_msg.pose.orientation.w;
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.scale.z = 0.25;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.5;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    return marker;
}
std::vector<float> FrontierDB::GetCalculatedWaypoint(Contour c){

    // Stop exploration when there are no longer any frontier goals. 
    if (frontier_goals.size() == 0)
    {
        calculated_waypoint_ = c.GetRobotPosition();
    }
   return calculated_waypoint_;
}

} // End of FrontierDB Class 

    
