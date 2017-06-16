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

  tf::TransformBroadcaster odom_broadcaster;
  
  // Initialize Variables
  int displayData;
  double positionCovariance, orientationCovariance, velocityCovariance, rotationRateCovariance;
  int addNoise;
  ros::Time timeCur, timePrev;
  int initialize, initCount;
  double initTime;
  double xInit, yInit, zInit, phiInit, theInit, psiInit;
  double x_, y_, z_, phi_, the_, psi_;
  double xOdom_, yOdom_, zOdom_, phiOdom_, theOdom_, psiOdom_;

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
    addNoise = 0;
    if (positionCovariance!=0.0 || orientationCovariance!=0.0 || velocityCovariance!=0.0 || rotationRateCovariance!=0.0) {
      addNoise = 1;
    }
    timeCur = ros::Time::now();
    timePrev = ros::Time::now();
    initialize = 1; 
    initCount = 0;
    initTime = 0.0;
    xInit = 0.0; yInit = 0.0; zInit = 0.0; phiInit = 0.0; theInit = 0.0; psiInit = 0.0;
    x_ = 0.0; y_ = 0.0; z_ = 0.0; phi_ = 0.0; the_ = 0.0; psi_ = 0.0;
    xOdom_ = 0.0; yOdom_ = 0.0; zOdom_ = 0.0; phiOdom_ = 0.0; theOdom_ = 0.0; psiOdom_ = 0.0;

    // std::cout << "Listening to Optitrack for 1 Seconds" << std::endl;
  }

  void mocapCallback(const geometry_msgs::PoseStamped& msg)
  {
    // Extract Data
    double xCur = msg.pose.position.x;
    double yCur = msg.pose.position.y;
    double zCur = msg.pose.position.z;
    double qx = msg.pose.orientation.x;
    double qy = msg.pose.orientation.y;
    double qz = msg.pose.orientation.z;
    double qw = msg.pose.orientation.w;

    // Conversion to Euler Angles
    double phiCur, theCur, psiCur;
    tf::Quaternion q(qx, qy, qz, qw);
    tf::Matrix3x3 m(q);
    m.getRPY(phiCur, theCur, psiCur);

    // Initialize Starting Robot Pose
    if (initialize==1) {
      timeCur = ros::Time::now();
      // Add Poses
      double dt = (timeCur-timePrev).toSec();
      initTime = initTime + dt;
      if ( initTime<1.0 ) {
        initCount++;
        xInit = xInit + xCur;
        yInit = yInit + yCur;
        zInit = zInit + zCur;
        phiInit = phiInit + phiCur;
        theInit = theInit + theCur;
        psiInit = psiInit + psiCur;
      }
      else {
        // Average Poses
        xInit = xInit/double(initCount);
        yInit = yInit/double(initCount);
        zInit = zInit/double(initCount);
        phiInit = phiInit/double(initCount);
        theInit = theInit/double(initCount);
        psiInit = psiInit/double(initCount);
        timePrev = ros::Time::now();
        initialize = 0;
        std::cout << "Initial Pose:" << endl << "x: " << xInit << endl << "y: " << yInit << endl << "psi: " << psiInit << endl << endl;
        // Set Previous Values
        timePrev = ros::Time::now();
        x_ = xInit;
        y_ = yInit;
        z_ = zInit;
        phi_ = phiInit;
        the_ = theInit;
        psi_ = psiInit;
        xOdom_ = xInit;
        yOdom_ = yInit;
        zOdom_ = 0.0;
        phiOdom_ = 0.0;
        theOdom_ = 0.0;
        psiOdom_ = psiInit;
      }
    }

    else { // Compute Odometry
      // Current Time
      timeCur = ros::Time::now();

      // Add Noise to Velocity Estimate
      double dt = (timeCur-timePrev).toSec();
      double vx = (xCur-x_)*dt;
      double vy = (yCur-y_)*dt;
      // double vz = (zCur-z_)*dt;
      // double vphi = (phiCur-phi_)*dt;
      // double vthe = (theCur-the_)*dt;
      double vpsi = (psiCur-psi_)*dt;
      if (addNoise==1) {
        const double mu = 0.0;
        std::random_device rd;
        std::mt19937 generator(rd());
        std::normal_distribution<double> vxNoise(mu,velocityCovariance);
        std::normal_distribution<double> vyNoise(mu,velocityCovariance);
        std::normal_distribution<double> vpsiNoise(mu,rotationRateCovariance);
        vx = vx + vxNoise(generator);
        vy = vy + vyNoise(generator);
        vpsi = vpsi + vpsiNoise(generator);
      }

      // Compute Velocity in Body Frame (/base_link)
      // v^b = R_bg v^g
      double vxLocal = cos(psiOdom_)*vx + sin(psiOdom_)*vy;
      double vyLocal = -sin(psiOdom_)*vx + cos(psiOdom_)*vy;

      // Add Noise in Planar Coordinates Only
      double xOdom = xOdom_;
      double yOdom = yOdom_;
      double psiOdom = psiOdom_;
      if (addNoise==1) {
        const double mu = 0.0;
        std::random_device rd;
        std::mt19937 generator(rd());
        std::normal_distribution<double> xNoise(mu,positionCovariance);
        std::normal_distribution<double> yNoise(mu,positionCovariance);
        std::normal_distribution<double> psiNoise(mu,orientationCovariance);
        xOdom = xOdom + xNoise(generator);
        yOdom = yOdom + yNoise(generator);
        psiOdom = psiOdom + psiNoise(generator);
      }
      xOdom = xOdom + vx/dt;
      yOdom = yOdom + vy/dt;
      psiOdom = psiOdom + vpsi/dt;

      // Publish Odometry Transform
      geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(psiOdom);
      geometry_msgs::TransformStamped odom_trans;
      odom_trans.header.stamp = timeCur;
      odom_trans.header.frame_id = "/odom_frame";
      odom_trans.child_frame_id = "/base_link";
      odom_trans.transform.translation.x = xOdom;
      odom_trans.transform.translation.y = yOdom;
      odom_trans.transform.translation.z = 0.0;
      odom_trans.transform.rotation = odom_quat;
      odom_broadcaster.sendTransform(odom_trans);
      usleep(1000);

      // Set Final Planar Pose
      nav_msgs::Odometry odomMsg;
      odomMsg.header.stamp = timeCur;
      odomMsg.header.frame_id = "/odom_frame";
      odomMsg.pose.pose.position.x = xOdom;
      odomMsg.pose.pose.position.y = yOdom;
      odomMsg.pose.pose.position.z = 0.0;
      odomMsg.pose.pose.orientation = odom_quat;

      // Set Final Planar Velocity
      odomMsg.child_frame_id = "/base_link";
      odomMsg.twist.twist.linear.x = vxLocal;
      odomMsg.twist.twist.linear.y = vyLocal;
      odomMsg.twist.twist.linear.z = 0.0;
      odomMsg.twist.twist.angular.x = 0.0;
      odomMsg.twist.twist.angular.y = 0.0;
      odomMsg.twist.twist.angular.z = vpsi;

      // Publish Odometry Message
      odom_pub_.publish(odomMsg);

      // Set Previous Values
      timePrev = timeCur;
      x_ = xCur;
      y_ = yCur;
      z_ = zCur;
      phi_ = psiCur;
      the_ = theCur;
      psi_ = psiCur;
      xOdom_ = xOdom;
      yOdom_ = yOdom;
      zOdom_ = 0.0;
      phiOdom_ = 0.0;
      theOdom_ = 0.0;
      psiOdom_ = psiOdom;

      // Display Data
      if (displayData==1) {
        std::cout << "Ground Truth Pose:" << endl << "x: " << xCur << endl << "y: " << yCur << endl << "psi: " << psiCur << endl << endl;
        std::cout << "Odometry Pose:" << endl << "x: " << xOdom << endl << "y: " << yOdom << endl << "psi: " << psiOdom << endl << endl;
        std::cout << "Body Frame Velocity:" << endl << "vxLocal: " << vxLocal << endl << "vyLocal: " << vyLocal << endl << "vpsi: " << vpsi << endl << endl;
      }

    } // and if not initializing

  } // end processMocap

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

