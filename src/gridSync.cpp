#include <stdio.h>
#include <signal.h>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include "nav_msgs/OccupancyGrid.h"


// Publishers
ros::Publisher _pubMap;
ros::Publisher _pubPointCloud;

// Msg Data
nav_msgs::OccupancyGrid _msgMap;
sensor_msgs::PointCloud2 _msgPointCloud;



void SigintHandler(int sig)
{
  ROS_INFO("Shutting down navigation ...");
  ros::shutdown();
}


void InitializeMapMsg(){

  // Initialize grid map
  _msgMap.header.frame_id="gridMap";
  _msgMap.header.stamp = ros::Time::now(); 
  _msgMap.info.resolution = 1;         // float32
  _msgMap.info.width      = 10;           // uint32
  _msgMap.info.height     = 10;           // uint32
  
  int p[_msgMap.info.width*_msgMap.info.height] = {
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
  std::vector<signed char> a(p, p+_msgMap.info.width*_msgMap.info.height);
  _msgMap.data = a;
}

void SynthesizePointCloudMsg(){
  //Initialize a point cloud
  _msgPointCloud.header.stamp = ros::Time::now(); 
  _msgPointCloud.header.frame_id = "PointCloud";
  _msgPointCloud.height = 3;
  _msgPointCloud.width = 5;
  _msgPointCloud.point_step = 1;  // 1 point is 1 byte
  _msgPointCloud.row_step = 5;    // width x point_step

  unsigned int p[_msgPointCloud.width*_msgPointCloud.height] = {
    0, 0, 0, 0, 1,
    0, 0, 0, 0, 1,
    0, 1, 0, 0, 1 };   
  std::vector<unsigned char> a(p, p+_msgPointCloud.width*_msgPointCloud.height);
  _msgPointCloud.data = a;
}


void timerCallback(const ros::TimerEvent&){
  SynthesizePointCloudMsg();
  _pubPointCloud.publish(_msgPointCloud);
}



int main(int argc, char * argv[]) {

  ros::init(argc, argv, "gridSync");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  signal(SIGINT, SigintHandler);

  _pubMap = nh.advertise<nav_msgs::OccupancyGrid>("/GridMap", 1);
  _pubPointCloud = nh.advertise<sensor_msgs::PointCloud2>("/cloud", 1);


  // Initialzation
  InitializeMapMsg();
  




  double dTimeStepSec = 1;  // 1 Hz
  ros::Timer timer = nh.createTimer(ros::Duration(dTimeStepSec), timerCallback);


  ROS_INFO_STREAM("GridSync node has started.");
  ros::spin();
  return 0;
}

