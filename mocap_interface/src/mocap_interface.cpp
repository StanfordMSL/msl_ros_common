/*--------------------------------------------------------------------------
 
 File Name:         mocap_interface.cpp
 Date Created:      2017/06/08
 Date Modified:     2017/06/08
 
 Author:            Eric Cristofalo
 Contact:           eric.cristofalo@gmail.com
 
 Description:       ROS node for reading and transforming raw mocap_optitrack output
 
 -------------------------------------------------------------------------*/

#include <iostream>
#include <stdio.h>
#include <math.h>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>

using namespace std;

class mocapInterfaceClass {
    
  ros::NodeHandle nh_;
  
  ros::Subscriber process_mocap_sub_;

  ros::Publisher pose_out_pub_;
  geometry_msgs::Twist poseOutMsg;
  
  // Initialize Variables
  int frameIndex, displayData;
    
public:
  mocapInterfaceClass(int a, int b) {

    // Subscribe to Optitrack Topics
    process_mocap_sub_ = nh_.subscribe("/robot/pose", 1, &mocapInterfaceClass::mocapCallback, this);

    // Publish Ground Robot Ground Truth
    pose_out_pub_ = nh_.advertise<geometry_msgs::Twist>("/robot/twist", 1);

    // Initialize Variables
    frameIndex = a;
    displayData = b;
  }

  void mocapCallback(const geometry_msgs::PoseStamped& msg)
  {

    // Coordinate Frame Conversion
    float x, y, z, qx, qy, qz, qw;
    switch ( frameIndex ) {
      // // XY-Ground Plane, -Z Up Coordinate System Coordinate System
      // case 1:
      //   x = msg.pose.position.x;
      //   y = msg.pose.position.y;
      //   z = msg.pose.position.z;
      //   qx = msg.pose.orientation.x;
      //   qy = msg.pose.orientation.y;
      //   qz = msg.pose.orientation.z;
      //   qw = msg.pose.orientation.w;
      // break;
      // // NED (north-east-down) axis defined in px4
      // case 2:
      //   x = msg.pose.position.x;
      //   y = msg.pose.position.y;
      //   z = msg.pose.position.z;
      //   qx = msg.pose.orientation.x;
      //   qy = msg.pose.orientation.y;
      //   qz = msg.pose.orientation.z;
      //   qw = msg.pose.orientation.w;
      // break;
      // Original Mocap Data
      default:
        x = msg.pose.position.x;
        y = msg.pose.position.y;
        z = msg.pose.position.z;
        qx = msg.pose.orientation.x;
        qy = msg.pose.orientation.y;
        qz = msg.pose.orientation.z;
        qw = msg.pose.orientation.w;
      break;
    }
    if (displayData==1) {
      std::cout << "Mocap Raw Pose:" << endl << "x: " << x << endl << "y: " << y << endl << "z: " << z << endl << "qx: " << qx << endl << "qy: " << qy << endl << "qz: " << qz << endl << "qw: " << qw << endl << endl; 
    }

    // Conversion
    tf::Quaternion q(qx, qy, qz, qw);
    tf::Matrix3x3 m(q);
    double phi, the, psi;
    m.getRPY(phi, the, psi);
    if (displayData==1) {
      std::cout << "Mocap Updated Pose:" << endl << "x: " << x << endl << "y: " << y << endl << "z: " << z << endl << "phi: " << phi << endl << "the: " << the << endl << "psi: " << psi << endl << endl; 
    }

    // Publish Final Estimation Message
    poseOutMsg.linear.x = x;
    poseOutMsg.linear.y = y;
    poseOutMsg.linear.z = z;
    poseOutMsg.angular.x = phi;
    poseOutMsg.angular.y = the;
    poseOutMsg.angular.z = psi;
    pose_out_pub_.publish(poseOutMsg);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mocap_interface");
  ros::NodeHandle nh("~");
  
  int frameIndex;
  nh.param<int>("COORDINATE_FRAME_INDEX", frameIndex, 0);
  int displayData;
  nh.param<int>("DISPLAY_DATA_BOOL", displayData, 0);

  mocapInterfaceClass poseEstimation(frameIndex, displayData);

  while (ros::ok()) {
    ros::spinOnce();
  }
  ros::shutdown();
  
  return 0;
}

