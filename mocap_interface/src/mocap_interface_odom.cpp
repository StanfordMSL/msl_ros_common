/*--------------------------------------------------------------------------
 
 File Name:         mocap_interface_tf.cpp
 Date Created:      2017/06/08
 Date Modified:     2017/06/09
 
 Author:            Eric Cristofalo
 Contact:           eric.cristofalo@gmail.com
 
 Description:       ROS node for outputting realtime odometry from Noisy Optitrack Pose
 
 -------------------------------------------------------------------------*/

#include <iostream>
#include <stdio.h>
#include <math.h>
#include <random>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

using namespace std;

class mocapInterfaceClass {
    
  ros::NodeHandle nh_;
  
  ros::Subscriber process_mocap_sub_;
  ros::Publisher odom_pub_;
  
  // Initialize Variables
  int displayData;
  double positionCovariance, orientationCovariance, velocityCovariance, rotationRateCovariance;
  ros::Time timeCur, timePrev;
  double xCur, yCur, zCur, phiCur, theCur, psiCur;
  double xPrev, yPrev, zPrev, phiPrev, thePrev, psiPrev;
    
public:
  mocapInterfaceClass(int a, double b, double c, double d, double e) {

    // Subscribe to Optitrack Topics
    process_mocap_sub_ = nh_.subscribe("/robot/pose", 10, &mocapInterfaceClass::mocapCallback, this);

    // Publish Odometry Message
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/robot/odom", 10);

    // Initialize Variables
    displayData = a;
    positionCovariance = b;
    orientationCovariance = c;
    velocityCovariance = d;
    rotationRateCovariance = e;
    timeCur = ros::Time::now();
    timePrev = ros::Time::now();
    xCur = 0.0; yCur = 0.0; zCur = 0.0; phiCur = 0.0; theCur = 0.0; psiCur = 0.0;
    xPrev = 0.0; yPrev = 0.0; zPrev = 0.0; phiPrev = 0.0; thePrev = 0.0; psiPrev = 0.0;
  }

  void mocapCallback(const geometry_msgs::PoseStamped& msg)
  {
    // Extract Data
    double qx, qy, qz, qw;
    xCur = msg.pose.position.x;
    yCur = msg.pose.position.y;
    zCur = msg.pose.position.z;
    qx = msg.pose.orientation.x;
    qy = msg.pose.orientation.y;
    qz = msg.pose.orientation.z;
    qw = msg.pose.orientation.w;

    // Conversion to Euler Angles
    tf::Quaternion q(qx, qy, qz, qw);
    tf::Matrix3x3 m(q);
    m.getRPY(phiCur, theCur, psiCur);

    if (xPrev!=0.0 && yPrev!=0.0) {

      // Current Time
      timeCur = ros::Time::now();

      // Add Noise in Planar Coordinates Only
      const double mean = 0.0;
      std::default_random_engine generator(time(0));
      std::normal_distribution<double> xNoise(mean,positionCovariance);
      std::normal_distribution<double> yNoise(mean,positionCovariance);
      std::normal_distribution<double> psiNoise(mean,orientationCovariance);
      double xOdom = xCur + xNoise(generator);
      double yOdom = yCur + yNoise(generator);
      double psiOdom = psiCur + psiNoise(generator);

      // Publish Odometry Transform
      geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(psiOdom);
      geometry_msgs::TransformStamped odom_trans;
      odom_trans.header.stamp = timeCur;
      odom_trans.header.frame_id = "odom_frame";
      odom_trans.child_frame_id = "base_link";
      odom_trans.transform.translation.x = xOdom;
      odom_trans.transform.translation.y = yOdom;
      odom_trans.transform.translation.z = 0.0;
      odom_trans.transform.rotation = odom_quat;
      tf::TransformBroadcaster odom_broadcaster;
      odom_broadcaster.sendTransform(odom_trans);

      // Set Final Planar Pose
      nav_msgs::Odometry odomMsg;
      odomMsg.header.stamp = timeCur;
      odomMsg.header.frame_id = "odom_frame";
      odomMsg.pose.pose.position.x = xOdom;
      odomMsg.pose.pose.position.y = yOdom;
      odomMsg.pose.pose.position.z = 0.0;
      odomMsg.pose.pose.orientation = odom_quat;

      // Add Noise to Velocity Estimate
      double dt = (timeCur-timePrev).toSec();
      double vx = (xCur-xPrev)*dt;
      double vy = (yCur-yPrev)*dt;
      // double vz = (zCur-zPrev)*dt;
      // double vphi = (phiCur-phiPrev)*dt;
      // double vthe = (theCur-thePrev)*dt;
      double vpsi = (psiCur-psiPrev)*dt;
      std::normal_distribution<double> vxNoise(mean,velocityCovariance);
      std::normal_distribution<double> vyNoise(mean,velocityCovariance);
      std::normal_distribution<double> vpsiNoise(mean,rotationRateCovariance);
      vx = vx + vxNoise(generator);
      vy = vy + vyNoise(generator);
      vpsi = vpsi + vpsiNoise(generator);

      // Set Final Planar Velocity
      odomMsg.child_frame_id = "base_link";
      odomMsg.twist.twist.linear.x = vx;
      odomMsg.twist.twist.linear.y = vy;
      odomMsg.twist.twist.linear.z = 0.0;
      odomMsg.twist.twist.angular.x = 0.0;
      odomMsg.twist.twist.angular.y = 0.0;
      odomMsg.twist.twist.angular.z = vpsi;

      // Publish Odometry Message
      odom_pub_.publish(odomMsg);

      // Set Previous Values
      xPrev = xCur;
      yPrev = yCur;
      zPrev = zCur;
      phiPrev = phiCur;
      thePrev = theCur;
      psiPrev = psiCur;
      timePrev = timeCur;

      // Display Data
      if (displayData==1) {
        std::cout << "Planar Pose:" << endl << "x: " << xCur << endl << "y: " << yCur << endl << "psi: " << psiCur << endl << endl;
        std::cout << "Noisy Odometry:" << endl << "x: " << xOdom << endl << "y: " << yOdom << endl << "psi: " << psiOdom << endl << endl;
        std::cout << "Velocity:" << endl << "vx: " << vx << endl << "vy: " << vy << endl << "vpsi: " << vpsi << endl << endl;
      }

    }

    else {

      // Set Previous Values
      xPrev = xCur;
      yPrev = yCur;
      zPrev = zCur;
      phiPrev = phiCur;
      thePrev = theCur;
      psiPrev = psiCur;
      timePrev = timeCur;

    }

  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mocap_interface_odom");
  ros::NodeHandle nh("~");
  
  int displayData;
  nh.param<int>("DISPLAY_DATA_BOOL", displayData, 0);
  double positionCovariance;
  nh.param<double>("POSITION_COVARIANCE", positionCovariance, 0.0);
  double orientationCovariance;
  nh.param<double>("ORIENTATION_COVARIANCE", orientationCovariance, 0.0);
  double velocityCovariance;
  nh.param<double>("VELOCITY_COVARIANCE", velocityCovariance, 0.0);
  double rotationRateCovariance;
  nh.param<double>("ROTATION_RATE_COVARIANCE", rotationRateCovariance, 0.0);

  mocapInterfaceClass poseEstimation(displayData, positionCovariance, orientationCovariance, velocityCovariance, rotationRateCovariance);

  while (ros::ok()) {
    ros::spinOnce();
  }
  ros::shutdown();
  
  return 0;
}

