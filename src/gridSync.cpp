#include <stdio.h>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"

int main(int argc, char * argv[]) {

  ros::init(argc, argv, "gridSync");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  ros::Publisher pubMap = nh.advertise<nav_msgs::OccupancyGrid>("/gridMap", 1);
  nav_msgs::OccupancyGrid msgMap;

  msgMap.header.frame_id="gridMap";
  msgMap.header.stamp = ros::Time::now(); 
  msgMap.info.resolution = 1;         // float32
  msgMap.info.width      = 10;           // uint32
  msgMap.info.height     = 10;           // uint32
  
  int p[msgMap.info.width*msgMap.info.height] = {
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
  std::vector<signed char> a(p, p+msgMap.info.width*msgMap.info.height);
  msgMap.data = a;


  // while (ros::ok())
  // {
  //     pubMap.publish(msgMap);
  // }



  ros::shutdown();
  return 0;
}

