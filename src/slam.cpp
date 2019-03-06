#include <stdio.h>
#include <signal.h>
#include "ros/ros.h"
// #include "cartographer/mapping/2d/probability_grid.h"
#include <geometry_msgs/Pose2D.h>
#include "sensor_msgs/PointCloud2.h"



//-----------------------------------
// Global variables
//-----------------------------------
// Publishers
ros::Publisher _pubRobotPose;

// Msg Data
sensor_msgs::PointCloud2 _msgPointCloud;
geometry_msgs::Pose2D _msgEstimatedRobotPose;



void SigintHandler(int sig)
{
  ROS_INFO("Shutting down ...");
  ros::shutdown();
}


void CreateProbabilityMap(){}
void UpdateProbabilityMap(){}
void EstimateRobotPose(){}
void UpdateEstimatedRobotPoseMsg(){}
void DetectLoopClosure(){}
void ConductBundleAdjustment(){}


void PointCloudCallback(const sensor_msgs::PointCloud2 msgData){
  _msgPointCloud.header = msgData.header;
  _msgPointCloud.height = msgData.height;
  _msgPointCloud.width = msgData.width;
  _msgPointCloud.point_step = msgData.point_step;  // 1 point is 1 byte
  _msgPointCloud.row_step = msgData.row_step;    // width x point_step
  _msgPointCloud.data = msgData.data;


  // [TODO] Update the point cloud data to probability grid 
  UpdateProbabilityMap();

  // [TODO] Estimate robot pose with scan matcher
  EstimateRobotPose();

}


void timerCallback(const ros::TimerEvent&){

}

int main(int argc, char * argv[]) {

  ros::init(argc, argv, "slam");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  signal(SIGINT, SigintHandler);


  // [TODO] Create probability map
  CreateProbabilityMap();



  


  ros::Subscriber sub1 = nh.subscribe("/cloud", 10, PointCloudCallback);

  double dTimeStepSec = 1;  // 1 Hz
  ros::Timer timer = nh.createTimer(ros::Duration(dTimeStepSec), timerCallback);


  ROS_INFO_STREAM("Slam node has started.");
  ros::spin();
  return 0;
}

