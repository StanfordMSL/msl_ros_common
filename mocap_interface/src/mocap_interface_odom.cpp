/*--------------------------------------------------------------------------
 
 File Name:         mocap_interface_odom.cpp
 Date Created:      2017/06/08
 Date Modified:     2017/09/05
 
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
  int displayData, publishAcceleration;
  double positionCovariance, orientationCovariance, velocityCovariance, rotationRateCovariance;
  VectorXd covariance;
  int addNoise;
  ros::Time timeCur, timePrev;
  int initialize, initCount, totalDifferenceCount, sumDifference, sumDifferenceCount;
  double initTime;
  VectorXd xInit, x, x_, v, v_, a;
  MatrixXd R_wr, R_phi, R_the, R_psi;

public:
  mocapInterfaceClass(int in_01, int in_02, vector<double> in_03) {

    // Subscribe to Optitrack Topics
    process_mocap_sub_ = nh_.subscribe("/robot/pose", 10, &mocapInterfaceClass::mocapCallback, this);

    // Publish Odometry Message
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/robot/odom", 10);

    // Initialize Variables
    displayData = in_01;
    publishAcceleration = in_02;
    covariance = VectorXd::Zero(18);
    for ( int i=0; i<18; i++ ) {
      covariance(i) = in_03[i];
    }

    // Publich Acceleration Message
    if ( publishAcceleration ) {
      accel_pub_ = nh_.advertise<geometry_msgs::Accel>("/robot/accel", 10);
    }

    // Initialize Class Variables
    addNoise = 0;
    if ( (covariance.array() != 0.0).any() ) {
      addNoise = 1;
    }
    timeCur = ros::Time::now();
    timePrev = ros::Time::now();
    initialize = 1; 
    initCount = 0;
    initTime = 0.0;
    totalDifferenceCount = 5;
    sumDifference = 1;
    sumDifferenceCount = 0;
    xInit = VectorXd::Zero(6);
    x = VectorXd::Zero(6);
    x_ = VectorXd::Zero(6);
    v = VectorXd::Zero(6);
    v_ = VectorXd::Zero(6);
    a = VectorXd::Zero(6);
    R_the = MatrixXd::Zero(3,3);
    R_phi = MatrixXd::Zero(3,3);
    R_psi = MatrixXd::Zero(3,3);
    R_wr = MatrixXd::Zero(3,3);

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
      if ( initCount==0) {
        timePrev = ros::Time::now();
      }
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
        cout << "mocap_interface_odom: initial mean pose after " << double(initCount) << " iterations and " << initTime << " seconds: " << endl << xInit << endl;
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
        std::normal_distribution<float> noise_x_0(mu, covariance(0));
        std::normal_distribution<float> noise_x_1(mu, covariance(1));
        std::normal_distribution<float> noise_x_2(mu, covariance(2));
        std::normal_distribution<float> noise_x_3(mu, covariance(3));
        std::normal_distribution<float> noise_x_4(mu, covariance(4));
        std::normal_distribution<float> noise_x_5(mu, covariance(5));
        xNoise << noise_x_0(generator), 
                  noise_x_1(generator),
                  noise_x_2(generator),
                  noise_x_3(generator),
                  noise_x_4(generator),
                  noise_x_5(generator);
      }
      x = x + xNoise;

      // Running Sum Indicator for Velocity and Acceleration Differencing
      bool averageBool = false;
      double dt;
      if ( sumDifferenceCount<totalDifferenceCount ) {
        sumDifferenceCount ++;
      }
      else {
        // Compute ROS loop time
        timeCur = ros::Time::now();
        dt = (timeCur-timePrev).toSec();
        if ( dt>0.0 ) { // compute average with logical dt
          averageBool = true;
        }
        // Reset Counter
        timePrev = timeCur;
        sumDifferenceCount = 0;
      }

      // Current Position Difference for Velocity ---------- ---------- ---------- ---------- ----------
      if ( averageBool ) {

        // Average Poses
        v = 1.0/(dt*totalDifferenceCount)*(x-x_);

        // // Compute Velocity in Body Frame (/base_link)
        // // v^b = R_bg v^g
        // v.head(3) = R_wr.transpose()*v.head(3);

        // Add Velocity Noise
        VectorXd vNoise = VectorXd::Zero(6);
        if ( addNoise ) {
          const double mu = 0.0;
          std::random_device rd;
          std::mt19937 generator(rd());
          std::normal_distribution<float> noise_x_0(mu, covariance(6));
          std::normal_distribution<float> noise_x_1(mu, covariance(7));
          std::normal_distribution<float> noise_x_2(mu, covariance(8));
          std::normal_distribution<float> noise_x_3(mu, covariance(9));
          std::normal_distribution<float> noise_x_4(mu, covariance(10));
          std::normal_distribution<float> noise_x_5(mu, covariance(11));
          vNoise << noise_x_0(generator), 
                    noise_x_1(generator),
                    noise_x_2(generator),
                    noise_x_3(generator),
                    noise_x_4(generator),
                    noise_x_5(generator);
        }
        v = v + vNoise;
      } // end if not averaging velocity

      // Current Velocity Difference for Acceleration ---------- ---------- ---------- ---------- ----------
      if ( publishAcceleration && averageBool) {

        // Average Poses
        a = 1.0/(dt*totalDifferenceCount)*( (x-x_)/dt - v_ );

        // // Compute Acceleration in Body Frame (/base_link)
        // // a^b = R_bg a^g
        // a.head(3) = R_wr.transpose()*a.head(3);

        // Add Velocity Noise
        VectorXd aNoise = VectorXd::Zero(6);
        if ( addNoise ) {
          const double mu = 0.0;
          std::random_device rd;
          std::mt19937 generator(rd());
          std::normal_distribution<float> noise_x_0(mu, covariance(12));
          std::normal_distribution<float> noise_x_1(mu, covariance(13));
          std::normal_distribution<float> noise_x_2(mu, covariance(14));
          std::normal_distribution<float> noise_x_3(mu, covariance(15));
          std::normal_distribution<float> noise_x_4(mu, covariance(16));
          std::normal_distribution<float> noise_x_5(mu, covariance(17));
          aNoise << noise_x_0(generator), 
                    noise_x_1(generator),
                    noise_x_2(generator),
                    noise_x_3(generator),
                    noise_x_4(generator),
                    noise_x_5(generator);
        }
        a = a + aNoise;
      } // end if not averaging acceleration

      // Publish Odometry Transform
      geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromRollPitchYaw(x(3),x(4),x(5));
      geometry_msgs::TransformStamped odom_trans;
      odom_trans.header.stamp = timeCur;
      odom_trans.header.frame_id = "/odom_frame_est";
      odom_trans.child_frame_id = "/base_link_est";
      odom_trans.transform.translation.x = x(0);
      odom_trans.transform.translation.y = x(1);
      odom_trans.transform.translation.z = x(2);
      odom_trans.transform.rotation = odom_quat;
      odom_broadcaster.sendTransform(odom_trans);
      // Set Final Pose
      odomMsg.header.stamp = timeCur;
      odomMsg.header.frame_id = "/odom_frame_est";
      odomMsg.pose.pose.position.x = x(0);
      odomMsg.pose.pose.position.y = x(1);
      odomMsg.pose.pose.position.z = x(2);
      odomMsg.pose.pose.orientation = odom_quat;
      // Set Final Velocity
      odomMsg.child_frame_id = "/base_link_est";
      odomMsg.twist.twist.linear.x = v(0);
      odomMsg.twist.twist.linear.y = v(1);
      odomMsg.twist.twist.linear.z = v(2);
      odomMsg.twist.twist.angular.x = v(3);
      odomMsg.twist.twist.angular.y = v(4);
      odomMsg.twist.twist.angular.z = v(5);
      // Publish Odometry Message
      odom_pub_.publish(odomMsg);

      // Publish Acceleration Message
      if ( publishAcceleration ) {
        accelMsg.linear.x = a(0);
        accelMsg.linear.y = a(1);
        accelMsg.linear.z = a(2);
        accelMsg.angular.x = a(3);
        accelMsg.angular.y = a(4);
        accelMsg.angular.z = a(5);
        accel_pub_.publish(accelMsg);
      }

      // Set Previous Values For Differencing
      if ( averageBool ) {
        timePrev = timeCur;
        x_ = x;
        v_ = v;
      }

      // Display Data
      if ( displayData ) {
        // cout << "Odometry Message: " << endl << odomMsg << endl;
        cout << "Pose:" << endl << x << endl;
        cout << "Velocity:" << endl << v << endl;
        if ( publishAcceleration ) {
          cout << "Acceleration:" << endl << a << endl;
        }
        cout << "------------------------------" << endl;
      }

    } // end if not initializing

  } // end processMocap

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mocap_interface_odom");
  ros::NodeHandle nh("~");
  
  int display_data_flag;
  nh.param<int>("display_data_flag", display_data_flag, 0);

  int publish_acceleration_flag;
  nh.param<int>("publish_acceleration_flag", publish_acceleration_flag, 0);

  std::vector<double> covariance;
  nh.getParam( "covariance", covariance );
  if (!nh.hasParam("covariance")) {
    covariance = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    // [x,y,z,phi,the,psi,v_x,v_y,v_z,v_phi,v_the,v_psi,a_x,a_y,a_z,a_phi,a_the,a_psi]
  }

  double positionCovariance;
  nh.param<double>("position_covariance", positionCovariance, 0.0);
  double orientationCovariance;
  nh.param<double>("orientation_covariance", orientationCovariance, 0.0);
  double velocityCovariance;
  nh.param<double>("velocity_covariance", velocityCovariance, 0.0);
  double rotationRateCovariance;
  nh.param<double>("rotation_rate_covariance", rotationRateCovariance, 0.0);

  mocapInterfaceClass poseEstimation(
    display_data_flag, 
    publish_acceleration_flag, 
    covariance
  );

  while (ros::ok()) {
    ros::spinOnce();
  }
  ros::shutdown();
  
  return 0;
}

