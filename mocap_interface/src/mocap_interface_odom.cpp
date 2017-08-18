/*--------------------------------------------------------------------------
 
 File Name:         mocap_interface_tf.cpp
 Date Created:      2017/06/08
 Date Modified:     2017/08/18
 
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
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Accel.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

class mocapInterfaceClass {
    
  ros::NodeHandle nh_;
  
  ros::Subscriber process_mocap_sub_;
  
  nav_msgs::Odometry odomMsg;
  ros::Publisher odom_pub_;
  tf::TransformBroadcaster odom_broadcaster;

  geometry_msgs::Accel accelMsg;
  ros::Publisher accel_pub_;
  
  // Initialize Variables
  int displayData;
  double positionCovariance, orientationCovariance, velocityCovariance, rotationRateCovariance;
  int addNoise;
  ros::Time timeCur, timePrev, timeACur, timeAPrev;
  int initialize, initCount, sumVel, sumVelCount;
  double initTime;
  VectorXd xInit, x, x_, v, v_, a, a_;
  MatrixXd R_wr, R_phi, R_the, R_psi;

public:
  mocapInterfaceClass(int in_01, double in_02, double in_03, double in_04, double in_05) {

    // Subscribe to Optitrack Topics
    process_mocap_sub_ = nh_.subscribe("/robot/pose", 10, &mocapInterfaceClass::mocapCallback, this);

    // Publish Odometry Message
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/robot/odom", 10);

    // Publich Acceleration Message
    accel_pub_ = nh_.advertise<geometry_msgs::Accel>("/robot/accel", 10);

    // Initialize Variables
    displayData = in_01;
    positionCovariance = in_02;
    orientationCovariance = in_03;
    velocityCovariance = in_04;
    rotationRateCovariance = in_05;
    addNoise = 0;
    if (positionCovariance!=0.0 || orientationCovariance!=0.0 || velocityCovariance!=0.0 || rotationRateCovariance!=0.0) {
      addNoise = 1;
    }
    timeCur = ros::Time::now();
    timePrev = ros::Time::now();
    timeACur = ros::Time::now();
    timeAPrev = ros::Time::now();
    initialize = 1; 
    initCount = 0;
    initTime = 0.0;
    sumVel = 1;
    sumVelCount = 0;
    xInit = VectorXd::Zero(6);
    x = VectorXd::Zero(6);
    x_ = VectorXd::Zero(6);
    v = VectorXd::Zero(6);
    v_ = VectorXd::Zero(6);
    a = VectorXd::Zero(6);
    a_ = VectorXd::Zero(6);

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
    R_phi <<  1.0    ,   0.0         ,   0.0         ,
              0.0    ,   cos(phiCur) ,   -sin(phiCur),
              0.0    ,   sin(phiCur) ,   cos(phiCur) ;
    R_the <<  cos(theCur)    ,   0.0 ,   sin(theCur) ,
              0.0            ,   1.0 ,   0.0         ,
              -sin(theCur)   ,   0.0 ,   cos(theCur) ;
    R_psi <<  cos(psiCur)    ,   -sin(psiCur),   0.0,
              sin(psiCur)    ,   cos(psiCur) ,   0.0,
              0.0            ,   0.0         ,   1.0;
    // Quad Orientation
    R_wr = R_psi*R_the*R_phi;

    // Initialize Starting Robot Pose
    VectorXd xTemp = VectorXd::Zero(6);
    xTemp << xCur, yCur, zCur, phiCur, theCur, psiCur;
    if ( initialize ) {
      timeCur = ros::Time::now();
      // Add Poses
      double dt = (timeCur-timePrev).toSec();
      initTime = initTime + dt;
      if ( initTime<1.0 ) {
        initCount++;
        xInit = xInit + xTemp;
      }
      else {
        // Average Poses
        xInit = xInit/double(initCount);
        timePrev = ros::Time::now();
        initialize = 0;
        std::cout << "Initial Pose:" << endl << xInit << "with iterations: " << endl << double(initCount) << endl;
        // Set Previous Values
        timePrev = ros::Time::now();
        x_ = xInit;
      }
    }

    else { // Compute Position Estimate

      // Current Position
      x = xTemp;

      // Add Position Noise
      VectorXd xNoise = VectorXd::Zero(6);
      if ( addNoise ) {
        const double mu = 0.0;
        std::random_device rd;
        std::mt19937 generator(rd());
        std::normal_distribution<float> noise_x_1(mu, positionCovariance);
        std::normal_distribution<float> noise_x_2(mu, positionCovariance);
        std::normal_distribution<float> noise_x_3(mu, positionCovariance);
        std::normal_distribution<float> noise_v_1(mu, orientationCovariance);
        std::normal_distribution<float> noise_v_2(mu, orientationCovariance);
        std::normal_distribution<float> noise_v_3(mu, orientationCovariance);
        xNoise << noise_x_1(generator), 
                  noise_x_2(generator),
                  noise_x_3(generator),
                  noise_v_1(generator),
                  noise_v_2(generator),
                  noise_v_3(generator);
      }
      x = x + xNoise;

      // Current Position Difference ---------- ---------- ---------- ---------- ----------
      VectorXd v_temp = (x - x_);

      // Running Sum for Velocity
      int totalCount = 5;
      if ( sumVelCount<totalCount ) {
        v_ = v_ + v_temp;
        sumVelCount ++;
      }
      else {

        // Current Time
        timeCur = ros::Time::now();
        double dt = (timeCur-timePrev).toSec();

        // Average Poses
        v = v_/dt;

        // Compute Velocity in Body Frame (/base_link)
        // v^b = R_bg v^g
        v.head(3) = R_wr.transpose()*v.head(3);

        // Add Velocity Noise
        VectorXd vNoise = VectorXd::Zero(6);
        if ( addNoise ) {
          const double mu = 0.0;
          std::random_device rd;
          std::mt19937 generator(rd());
          std::normal_distribution<float> noise_x_1(mu, velocityCovariance);
          std::normal_distribution<float> noise_x_2(mu, velocityCovariance);
          std::normal_distribution<float> noise_x_3(mu, velocityCovariance);
          std::normal_distribution<float> noise_v_1(mu, rotationRateCovariance);
          std::normal_distribution<float> noise_v_2(mu, rotationRateCovariance);
          std::normal_distribution<float> noise_v_3(mu, rotationRateCovariance);
          vNoise << noise_x_1(generator), 
                    noise_x_2(generator),
                    noise_x_3(generator),
                    noise_v_1(generator),
                    noise_v_2(generator),
                    noise_v_3(generator);
        }
        v = v + vNoise;

        // Reset Velocity Values
        timePrev = timeCur;
        sumVelCount = 0;
        v_ = VectorXd::Zero(6);

      } // end if not averaging velocity

      // Current Velocity Difference ---------- ---------- ---------- ---------- ----------
      VectorXd a_temp = (v - v_);

      // Running Sum for Acceleration
      if ( sumVelCount<totalCount ) {
        a_ = a_ + a_temp;
        sumVelCount ++;
      }
      else {

        // Current Time
        timeACur = ros::Time::now();
        double dt = (timeACur-timeAPrev).toSec();

        // Average Poses
        a = a_/dt;

        // Compute Acceleration in Body Frame (/base_link)
        // a^b = R_bg a^g
        a.head(3) = R_wr.transpose()*a.head(3);

        // // Add Velocity Noise
        // VectorXd vNoise = VectorXd::Zero(6);
        // if ( addNoise ) {
        //   const double mu = 0.0;
        //   std::random_device rd;
        //   std::mt19937 generator(rd());
        //   std::normal_distribution<float> noise_x_1(mu, velocityCovariance);
        //   std::normal_distribution<float> noise_x_2(mu, velocityCovariance);
        //   std::normal_distribution<float> noise_x_3(mu, velocityCovariance);
        //   std::normal_distribution<float> noise_v_1(mu, rotationRateCovariance);
        //   std::normal_distribution<float> noise_v_2(mu, rotationRateCovariance);
        //   std::normal_distribution<float> noise_v_3(mu, rotationRateCovariance);
        //   vNoise << noise_x_1(generator), 
        //             noise_x_2(generator),
        //             noise_x_3(generator),
        //             noise_v_1(generator),
        //             noise_v_2(generator),
        //             noise_v_3(generator);
        // }
        // v = v + vNoise;

        // Reset Velocity Values
        timeAPrev = timeACur;
        sumVelCount = 0;
        a_ = VectorXd::Zero(6);

      } // end if not averaging velocity

      // Publish Odometry Transform
      geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromRollPitchYaw(x(3),x(4),x(5));
      geometry_msgs::TransformStamped odom_trans;
      odom_trans.header.stamp = timeCur;
      odom_trans.header.frame_id = "odom_frame";
      odom_trans.child_frame_id = "base_link";
      odom_trans.transform.translation.x = x(0);
      odom_trans.transform.translation.y = x(1);
      odom_trans.transform.translation.z = x(2);
      odom_trans.transform.rotation = odom_quat;
      odom_broadcaster.sendTransform(odom_trans);

      // Set Final Pose
      odomMsg.header.stamp = timeCur;
      odomMsg.header.frame_id = "odom_frame";
      odomMsg.pose.pose.position.x = x(0);
      odomMsg.pose.pose.position.y = x(1);
      odomMsg.pose.pose.position.z = x(2);
      odomMsg.pose.pose.orientation = odom_quat;

      // Set Final Velocity
      odomMsg.child_frame_id = "base_link";
      odomMsg.twist.twist.linear.x = v(0);
      odomMsg.twist.twist.linear.y = v(1);
      odomMsg.twist.twist.linear.z = v(2);
      odomMsg.twist.twist.angular.x = v(3);
      odomMsg.twist.twist.angular.y = v(4);
      odomMsg.twist.twist.angular.z = v(5);

      // Publish Odometry Message
      odom_pub_.publish(odomMsg);

      // Publish Acceleration Message
      accelMsg.linear.x = a(0);
      accelMsg.linear.y = a(1);
      accelMsg.linear.z = a(2);
      accelMsg.angular.x = a(3);
      accelMsg.angular.y = a(4);
      accelMsg.angular.z = a(5);
      accel_pub_.publish(accelMsg);

      // Set Previous Values
      x_ = x;
      v_ = v;

      // Display Data
      if ( displayData ) {
        std::cout << "Pose:" << endl << x << endl;
        std::cout << "Velocity:" << endl << v << endl;
      }

    } // end if not initializing

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

