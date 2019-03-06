#include <stdio.h>
#include <signal.h>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include "nav_msgs/OccupancyGrid.h"
#include <geometry_msgs/Pose2D.h>



//-----------------------------------
// Global variables
//-----------------------------------
// Publishers
ros::Publisher _pubGridMap;
ros::Publisher _pubPointCloud;
ros::Publisher _pubRobotPose;

// Msg Data
nav_msgs::OccupancyGrid _msgGridMap;
sensor_msgs::PointCloud2 _msgPointCloud;
geometry_msgs::Pose2D _msgRobotPose;


unsigned int _stepCounter = 0;
const unsigned int _pointCloudArray[26][15] = {   // List all point cloud data as measurements
  { 0, 0, 0, 0, 1,    0, 0, 0, 0, 1,    0, 1, 0, 0, 1 },
  { 0, 1, 0, 0, 1,    0, 0, 0, 0, 1,    0, 0, 0, 0, 1 },
  { 0, 0, 0, 0, 1,    0, 1, 0, 0, 1,    0, 0, 0, 0, 1 },
  { 0, 0, 0, 0, 0,    0, 0, 0, 0, 1,    0, 1, 0, 0, 1 },
  { 0, 1, 0, 0, 1,    0, 0, 0, 0, 0,    0, 0, 0, 0, 1 },
  { 0, 0, 0, 0, 1,    0, 1, 0, 0, 1,    0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 1,    0, 0, 0, 0, 1,    0, 1, 0, 0, 1 },
  { 1, 0, 1, 1, 1,    0, 0, 0, 0, 1,    0, 0, 0, 0, 1 },
  { 0, 0, 0, 0, 1,    0, 1, 0, 0, 0,    0, 0, 0, 0, 1 },
  { 0, 0, 0, 0, 1,    0, 0, 0, 0, 1,    0, 1, 0, 0, 0 },
  { 0, 1, 0, 0, 1,    0, 0, 0, 0, 1,    0, 0, 0, 0, 1 },
  { 0, 0, 0, 0, 1,    0, 1, 0, 0, 1,    0, 0, 0, 0, 1 },
  { 0, 0, 0, 0, 1,    0, 0, 0, 0, 1,    0, 1, 0, 0, 1 },
  { 0, 1, 1, 1, 1,    0, 0, 0, 0, 1,    0, 0, 0, 0, 1 },
  { 0, 0, 0, 0, 0,    0, 1, 0, 0, 1,    0, 0, 0, 0, 1 },
  { 0, 0, 0, 0, 0,    0, 0, 0, 0, 0,    0, 1, 0, 0, 1 },
  { 0, 1, 0, 0, 1,    0, 0, 0, 0, 0,    0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 1,    0, 1, 0, 0, 1,    0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 1,    0, 0, 0, 0, 1,    0, 1, 0, 0, 1 },
  { 1, 1, 1, 1, 1,    0, 0, 0, 0, 1,    0, 0, 0, 0, 1 },
  { 0, 0, 0, 0, 1,    0, 1, 0, 0, 1,    0, 0, 0, 0, 1 },
  { 0, 0, 0, 0, 1,    0, 0, 0, 0, 1,    0, 1, 0, 0, 1 },
  { 0, 1, 0, 0, 1,    0, 0, 0, 0, 1,    0, 0, 0, 0, 1 },
  { 0, 0, 0, 0, 0,    0, 1, 0, 0, 1,    0, 0, 0, 0, 1 },
  { 0, 0, 0, 0, 0,    0, 0, 0, 0, 0,    0, 1, 0, 0, 1 },
  { 1, 1, 1, 1, 1,    0, 0, 0, 0, 0,    0, 0, 0, 0, 0 }
};
const int _robotPoseArray[26][3] = {    // List all the robot poses for simplicity
  { 0, 2, 0 },
  { 1, 2, 0 },
  { 2, 2, 0 },
  { 3, 2, 0 },
  { 4, 2, 0 },
  { 5, 2, 0 },
  { 6, 2, 0 },
  { 7, 2, 0 },
  { 7, 2, -90 },
  { 7, 3, -90 },
  { 7, 4, -90 },
  { 7, 5, -90 },
  { 7, 6, -90 },
  { 7, 7, -90 },
  { 7, 7, -180 },
  { 6, 7, -180 },
  { 5, 7, -180 },
  { 4, 7, -180 },
  { 3, 7, -180 },
  { 2, 7, -180 },
  { 2, 7, 90 },
  { 2, 6, 90 },
  { 2, 5, 90 },
  { 2, 4, 90 },
  { 2, 3, 90 },
  { 2, 2, 90 }
};



void SigintHandler(int sig)
{
  ROS_INFO("Shutting down ...");
  ros::shutdown();
}


void InitializeMapMsg(){

  // Initialize grid map
  _msgGridMap.header.frame_id="gridMap";
  _msgGridMap.header.stamp = ros::Time::now(); 
  _msgGridMap.info.resolution = 1;         // float32
  _msgGridMap.info.width      = 10;           // uint32
  _msgGridMap.info.height     = 10;           // uint32
  
  const int p[_msgGridMap.info.width*_msgGridMap.info.height] = {
    1, 1, 1, 1, 1, 1, 0, 1, 1, 1,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    1, 0, 0, 1, 0, 0, 1, 0, 0, 1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    1, 0, 0, 1, 0, 0, 1, 0, 0, 1,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 1,    
    1, 1, 1, 1, 1, 1, 1, 0, 0, 1  };   // [0,100]
  std::vector<signed char> a(p, p+_msgGridMap.info.width*_msgGridMap.info.height);
  _msgGridMap.data = a;
}

void UpdateRobotPoseMsg(){
  
  _msgRobotPose.x = _robotPoseArray[_stepCounter][0];
  _msgRobotPose.y = _robotPoseArray[_stepCounter][1];
  _msgRobotPose.theta = _robotPoseArray[_stepCounter][2];
  
}

void SynthesizePointCloudMsg(){
  //Initialize a point cloud
  _msgPointCloud.header.stamp = ros::Time::now(); 
  _msgPointCloud.header.frame_id = "PointCloud";
  _msgPointCloud.height = 3;
  _msgPointCloud.width = 5;
  _msgPointCloud.point_step = 1;  // 1 point is 1 byte
  _msgPointCloud.row_step = 5;    // width x point_step

  std::vector<unsigned char> a(_pointCloudArray[_stepCounter], _pointCloudArray[_stepCounter]+_msgPointCloud.width*_msgPointCloud.height);
  _msgPointCloud.data = a;
  
}



void timerCallback(const ros::TimerEvent&){

  // For each iteration, 
  // 1. update the robot pose msg
  if(_stepCounter >= 26)  { _stepCounter = 2;  }
  UpdateRobotPoseMsg();

  // 2. update the point cloud
  SynthesizePointCloudMsg();
  _pubPointCloud.publish(_msgPointCloud);

  // 3. iterate the robot 
  _stepCounter++;
}



int main(int argc, char * argv[]) {

  ros::init(argc, argv, "gridSync");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  signal(SIGINT, SigintHandler);

  _pubGridMap = nh.advertise<nav_msgs::OccupancyGrid>("/GridMap", 10);
  _pubPointCloud = nh.advertise<sensor_msgs::PointCloud2>("/cloud", 10);


  // Initialization
  InitializeMapMsg();
  




  double dTimeStepSec = 1;  // 1 Hz
  ros::Timer timer = nh.createTimer(ros::Duration(dTimeStepSec), timerCallback);


  ROS_INFO_STREAM("GridSync node has started.");
  ros::spin();
  return 0;
}

