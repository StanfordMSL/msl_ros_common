/*--------------------------------------------------------------------------
 
 File Name:         mocap_interface_tf.cpp
 Date Created:      2017/06/08
 Date Modified:     2017/06/09
 
 Author:            Eric Cristofalo
 Contact:           eric.cristofalo@gmail.com
 
 Description:       ROS node for outputting realtime /tf topic from Optitrack Pose
 
 -------------------------------------------------------------------------*/

#include <iostream>
#include <stdio.h>
#include <math.h>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

using namespace std;

class mocapInterfaceClass {
    
  ros::NodeHandle nh_;
  
  ros::Subscriber process_mocap_sub_;
  
  // Initialize Variables
  string coordinateFrameID, childFrameID;
    
public:
  mocapInterfaceClass(string a, string b) {

    // Subscribe to Optitrack Topics
    process_mocap_sub_ = nh_.subscribe("/robot/pose", 1, &mocapInterfaceClass::mocapCallback, this);

    // Initialize Variables
    coordinateFrameID = a;
    childFrameID = b;
  }

  void mocapCallback(const geometry_msgs::PoseStamped& msg)
  {
    // Publish Final TF Message
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3( msg.pose.position.x,msg.pose.position.y,msg.pose.position.z ) );
    tf::Quaternion q(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w);
    transform.setRotation( q );
    br.sendTransform( tf::StampedTransform( transform, ros::Time::now(), coordinateFrameID, childFrameID ) );
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mocap_interface_tf");
  ros::NodeHandle nh("~");
  
  string coordinateFrameID;
  nh.param<string>("COORDINATE_FRAME_ID", coordinateFrameID, "world_frame");
  string childFrameID;
  nh.param<string>("CHILD_FRAME_ID", childFrameID, "child_link");

  mocapInterfaceClass poseEstimation(coordinateFrameID, childFrameID);

  while (ros::ok()) {
    ros::spinOnce();
  }
  ros::shutdown();
  
  return 0;
}

