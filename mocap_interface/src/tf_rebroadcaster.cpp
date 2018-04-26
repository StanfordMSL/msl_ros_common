/*--------------------------------------------------------------------------
 
 File Name:         tf_rebroadcaster.cpp
 Date Created:      2017/06/30
 Date Modified:     2017/09/06
 
 Author:            Eric Cristofalo
 Contact:           eric.cristofalo@gmail.com
 
 Description:       ROS node that broadcasts a ground truth tf from ground truth position
 
 -------------------------------------------------------------------------*/

#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

using namespace std;

class tf_rebroadcasterClass {
    
  ros::NodeHandle nh_;
  
  ros::Subscriber input_sub_;
  tf::TransformBroadcaster tf_broadcaster;
  
  // Initialize Variables
  int displayData, inputDataType;
  string source_frame, destination_frame;

public:
  tf_rebroadcasterClass(int in_01, int in_02, string in_03, string in_04) {

    // Initialize Variables
    displayData = in_01;
    inputDataType = in_02;
    source_frame = in_03;
    destination_frame = in_04;

    if ( inputDataType==0 ) { // geometry_msgs/PoseStamped
      // Subscribe to PoseStamped
      input_sub_ = nh_.subscribe("/robot", 10, &tf_rebroadcasterClass::poseStampedCallback, this);
    }
    else if ( inputDataType==1 ) { // nav_msgs/Odometry
// Subscribe to PoseStamped
      input_sub_ = nh_.subscribe("/robot", 10, &tf_rebroadcasterClass::odometryCallback, this);
    }
    else {
      cout << "tf_rebroadcaster error: no data type defined for input_data_type_index = " << inputDataType << endl;
    }

  }

  void odometryCallback(const nav_msgs::Odometry& msg)
  {
    // Extract Data
    double x = msg.pose.pose.position.x;
    double y = msg.pose.pose.position.y;
    double z = msg.pose.pose.position.z;
    double qx = msg.pose.pose.orientation.x;
    double qy = msg.pose.pose.orientation.y;
    double qz = msg.pose.pose.orientation.z;
    double qw = msg.pose.pose.orientation.w;
    // Broadcast Transform
    geometry_msgs::Quaternion ground_truth_quat;
    ground_truth_quat.x = qx;
    ground_truth_quat.y = qy;
    ground_truth_quat.z = qz;
    ground_truth_quat.w = qw;
    geometry_msgs::TransformStamped ground_truth_trans;
    ground_truth_trans.header.stamp = msg.header.stamp;
    ground_truth_trans.header.frame_id = source_frame;
    ground_truth_trans.child_frame_id = destination_frame;
    ground_truth_trans.transform.translation.x = x;
    ground_truth_trans.transform.translation.y = y;
    ground_truth_trans.transform.translation.z = z;
    ground_truth_trans.transform.rotation = ground_truth_quat;
    tf_broadcaster.sendTransform(ground_truth_trans);
    // Display Data
    if ( displayData ) {
      std::cout << "Odometry Position:" << endl << "x: " << x << endl << "y: " << y << endl << "z: " << z << endl;
    }
  } // end processMocap

  void poseStampedCallback(const geometry_msgs::PoseStamped& msg)
  {
    // Extract Data
    double x = msg.pose.position.x;
    double y = msg.pose.position.y;
    double z = msg.pose.position.z;
    double qx = msg.pose.orientation.x;
    double qy = msg.pose.orientation.y;
    double qz = msg.pose.orientation.z;
    double qw = msg.pose.orientation.w;
    // Broadcast Transform
    geometry_msgs::Quaternion ground_truth_quat;
    ground_truth_quat.x = qx;
    ground_truth_quat.y = qy;
    ground_truth_quat.z = qz;
    ground_truth_quat.w = qw;
    geometry_msgs::TransformStamped ground_truth_trans;
    ground_truth_trans.header.stamp = msg.header.stamp;
    ground_truth_trans.header.frame_id = source_frame;
    ground_truth_trans.child_frame_id = destination_frame;
    ground_truth_trans.transform.translation.x = x;
    ground_truth_trans.transform.translation.y = y;
    ground_truth_trans.transform.translation.z = z;
    ground_truth_trans.transform.rotation = ground_truth_quat;
    tf_broadcaster.sendTransform(ground_truth_trans);
    // Display Data
    if ( displayData ) {
      std::cout << "Pose Stamped Position:" << endl << "x: " << x << endl << "y: " << y << endl << "z: " << z << endl;
    }
  } // end processMocap

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tf_rebroadcaster");
  ros::NodeHandle nh("~");
  
  int display_data_flag;
  nh.param<int>("display_data_flag", display_data_flag, 0);
  int input_data_type_index;
  nh.param<int>("input_data_type_index", input_data_type_index, 0);

  string source_frame;
  nh.param<string>("source_frame", source_frame, "/world");
  string destination_frame;
  nh.param<string>("destination_frame", destination_frame, "/base_link");

  tf_rebroadcasterClass tf_rebroadcaster(display_data_flag, input_data_type_index, source_frame, destination_frame);

  while (ros::ok()) {
    ros::spinOnce();
  }
  ros::shutdown();
  
  return 0;
}

