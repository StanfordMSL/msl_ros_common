/*--------------------------------------------------------------------------
 
 File Name:         waypoint_quad_control.cpp
 Date Created:      2017/07/12
 Date Modified:     2017/08/01
 
 Author:            Eric Cristofalo
 Contact:           eric.cristofalo@gmail.com
 
 Description:       ROS node for controlling quads via waypoints in Gazebo with RotorS
 
 -------------------------------------------------------------------------*/

#include <iostream>
#include <stdio.h>
#include <math.h>
#include <random>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <sensor_msgs/Joy.h>

#include <Eigen/Dense>
#include <Eigen/Core>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

class waypointQuadControlClass {
  
  ros::NodeHandle nh_;
   
  ros::Subscriber quad1_sub_;
  ros::Subscriber quad2_sub_;
  geometry_msgs::Pose quad1PoseMsg;
  geometry_msgs::Pose quad2PoseMsg;
  ros::Subscriber y1_sub_;
  ros::Subscriber y2_sub_;
  ros::Subscriber joystick_sub_;

  ros::Publisher waypoint_quad1_control_pub_;
  ros::Publisher waypoint_quad2_control_pub_;
  
  // Initialize Variables
  int displayData, controllerInd;

  // initialize Filter Variables
  int manualControl, yawControlToggle, quadToggle, relativeToggle;
  int quadIndex;
  int quadButton, coordinateButton;
  VectorXd x_1, x_2, y_1, y_1_, y_2, y_2_, joyInput;
  MatrixXd R_wr1, R_wr2;
  ros::Time startT, curT;
    
public:
  waypointQuadControlClass( int in_01, int in_02 ) {

    // Subscribe to Robot Pose
    quad1_sub_ = nh_.subscribe("/quad1_pose", 1, &waypointQuadControlClass::quad1Callback, this);
    quad2_sub_ = nh_.subscribe("/quad2_pose", 1, &waypointQuadControlClass::quad2Callback, this);

    // Subscribe to Camera Measurements
    y1_sub_ = nh_.subscribe("/quad1_measurement", 1, &waypointQuadControlClass::measurement1Callback, this);
    y2_sub_ = nh_.subscribe("/quad2_measurement", 1, &waypointQuadControlClass::measurement2Callback, this);

    // Subscribe to Joystick Topic
    joystick_sub_ = nh_.subscribe("/joy", 1, &waypointQuadControlClass::joystickCallback, this);

    // Publish Waypoint Quad Control Message
    waypoint_quad1_control_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/quad1_command", 1);
    waypoint_quad2_control_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/quad2_command", 1);

    // Initialize Class Variables
    displayData = in_01;
    controllerInd = in_02; // 0 = joystick, 1 = xBox

    // Initialize Filter Variables
    manualControl = 0;
    yawControlToggle = 0;
    quadToggle = 0;
    relativeToggle = 1; // begin in relative coordinate control
    quadButton = 0;
    coordinateButton = 0;
    x_1 = VectorXd::Zero(6);
    x_2 = VectorXd::Zero(6);
    y_1 = VectorXd::Zero(3);
    y_1_ = VectorXd::Zero(3);
    y_2 = VectorXd::Zero(3);
    y_2_ = VectorXd::Zero(3);
    joyInput = VectorXd::Zero(6);
    R_wr1 = MatrixXd::Zero(3,3);
    R_wr2 = MatrixXd::Zero(3,3);
    startT = ros::Time::now();
    curT = ros::Time::now();

    quadIndex = 0;
  }

  void publishQuadCommand( int quadInd, ros::Publisher waypoint_quad_control_pub_, VectorXd x, VectorXd y, VectorXd& y_, MatrixXd R_wr ) {

    // Use Joystick Control
    double desiredAltitude = 1.0;
    if ( manualControl==1 ) {

      // Relative Coordinate Control
      VectorXd controlInput = joyInput.head(3);
      double k = 0.75;
      if ( relativeToggle==1 ) {
        controlInput = R_wr*controlInput;
      }

      // Apply Translation Command
      if ( quadToggle==quadInd ) {
        // Position Command
        x.head(2) = x.head(2) + k*controlInput.head(2);
        // Yaw Command
        // x(5) = x(5) + joyInput(5);
        // cout << "Quad 1 Manual Waypoint: " << endl << x << endl;
      }

    }
    else {

    }

    // Open Loop Position Control for Testing
   curT = ros::Time::now();
   if ( quadInd==0 ) {
     float t = (curT - startT).toSec();
     double k = 0.25;
     x(0) = x(0) + k*sin(0.25*double(t));
     x(1) = x(1) + k*cos(0.75*double(t));
   }
   if ( quadInd==1 ) {
     float t = (curT - startT).toSec();
     double k = 0.15;
     x(0) = x(0) + k*sin(0.75*double(t));
     x(1) = x(1) + k*cos(0.25*double(t));
   }

    // Yaw Command From Vision
    double yawControl = 0.0;
    double kp = 3.0;
    double kd = 10.0;
    if ( y(0)==0 && y(1)==0 && y(2)==0 ) {
      // // Ball is Not Visible
      // yawControl = kp;
    }
    else {
      yawControl = kp*y(1) + kd*(y(1) - y_(1));
    }
    // Manual Yaw Control Toggle
    if ( manualControl==1 && yawControlToggle==1 && quadToggle==quadInd ) {
      // Manual Yaw Control
      x(5) = x(5) + joyInput(5);
      // Manual Altitude Control
      x(2) = x(2) + joyInput(2);
    }
    else {
      // Default Yaw Control
      x(5) = x(5) + yawControl;
      // Hold Altitude
      x(2) = desiredAltitude;
    }
    // cout << "Proportional Gain: " << endl << kp*y(1) << endl;
    // cout << "Derivative Gain: " << endl << kd*(y(1) - y_(1)) << endl;
    // cout << "Quad 1 Vision Yaw Control: " << endl << x(5) << endl;

    // Save Old Measurements
    y_ = y;

    // Assemble Trajectory Point Message
    trajectory_msgs::MultiDOFJointTrajectoryPoint pointMsg;
    pointMsg.time_from_start = ros::Duration(0.1);
    pointMsg.transforms.resize(1);
    pointMsg.transforms[0].translation.x = x(0);
    pointMsg.transforms[0].translation.y = x(1);
    pointMsg.transforms[0].translation.z = x(2);
    geometry_msgs::Quaternion quat;
    quat = tf::createQuaternionMsgFromRollPitchYaw( 0.0, 0.0, x(5) );
    pointMsg.transforms[0].rotation.x = quat.x;
    pointMsg.transforms[0].rotation.y = quat.y;
    pointMsg.transforms[0].rotation.z = quat.z;
    pointMsg.transforms[0].rotation.w = quat.w;
    pointMsg.velocities.resize(1);
    pointMsg.accelerations.resize(1);

    // Publish Quad 1 Command
    trajectory_msgs::MultiDOFJointTrajectory wayPointMsg;
    wayPointMsg.header.stamp = ros::Time::now();
    wayPointMsg.header.frame_id = "base_link";
    wayPointMsg.points.push_back(pointMsg);
    waypoint_quad_control_pub_.publish(wayPointMsg);
    // if ( displayData ) {
    //   cout << "Joystick Control Message: " << pointMsg << endl;
    // }
  }

  void joystickCallback(const sensor_msgs::Joy& msg) {

    // Read Joystick Message
    if ( controllerInd==0 ) { // Joystick
      // Set Joystick Position
      joyInput(0) = msg.axes[1];
      joyInput(1) = msg.axes[0];
      joyInput(2) = msg.axes[3];
      joyInput(5) = msg.axes[2];
      // Set Control Toggles
      manualControl = msg.buttons[0];
      // if (abs(joyInput(0))>0.1 || abs(joyInput(1))>0.1 || abs(joyInput(2))>0.1 || abs(joyInput(5))>0.1) {
      //   manualControl = 1;
      // }
      // else { manualControl=0; }
      yawControlToggle = msg.buttons[1];
      // Quad Toggle
      if ( quadButton!=msg.buttons[4] && msg.buttons[4]==1 ) {
        quadButton = 1;
        quadToggle = quadToggle + quadButton;
        if ( quadToggle==2 ) { quadToggle = 0; }
        if (quadButton!=0) {
          if (quadToggle==0) { cout << "Controlling Quad 1" << endl; }
          else { cout << "Controlling Quad 2" << endl; }
        }
      }
      else if ( quadButton!=msg.buttons[4] && msg.buttons[4]==0 ) {
        quadButton = 0;
      }
      // Coordinate Frame Toggle
      if ( coordinateButton!=msg.buttons[2] && msg.buttons[2]==1 ) {
        coordinateButton = 1;
        relativeToggle = relativeToggle + coordinateButton;
        if ( relativeToggle==2 ) { relativeToggle = 0; }
        if (coordinateButton!=0) {
          if (relativeToggle==0) { cout << "Controlling in Global Coordinates" << endl; }
          else { cout << "Controlling in Relative Coordinates" << endl; }
        }
      }
      else if ( coordinateButton!=msg.buttons[2] && msg.buttons[2]==0 ) {
        coordinateButton = 0;
      }
    }
    else if ( controllerInd==1 ) { // Xbox Controller
      // Set Joystick Position
      joyInput(0) = msg.axes[4];
      joyInput(1) = msg.axes[3];
      joyInput(2) = msg.axes[1];
      // joyInput(5) = 0.5*(msg.axes[5]-msg.axes[2]);
      joyInput(5) = msg.axes[0];
      // Set Control Toggles
      if ( msg.axes[2]<1.0 && msg.axes[2]!=0.0 ) { manualControl = 1; }
      else { manualControl = 0; }
      if ( msg.axes[2]==-1.0 ) { yawControlToggle = 1; }
      else { yawControlToggle = 0; }
      // Quad Toggle
      if ( quadButton!=msg.buttons[2] && msg.buttons[2]==1 ) {
        quadButton = 1;
        quadToggle = quadToggle + quadButton;
        if ( quadToggle==2 ) { quadToggle = 0; }
        if (quadButton!=0) {
          if (quadToggle==0) { cout << "Controlling Quad 1" << endl; }
          else { cout << "Controlling Quad 2" << endl; }
        }
      }
      else if ( quadButton!=msg.buttons[2] && msg.buttons[2]==0 ) {
        quadButton = 0;
      }
      // Coordinate Frame Toggle
      if ( coordinateButton!=msg.buttons[0] && msg.buttons[0]==1 ) {
        coordinateButton = 1;
        relativeToggle = relativeToggle + coordinateButton;
        if ( relativeToggle==2 ) { relativeToggle = 0; }
        if (coordinateButton!=0) {
          if (relativeToggle==0) { cout << "Controlling in Global Coordinates" << endl; }
          else { cout << "Controlling in Relative Coordinates" << endl; }
        }
      }
      else if ( coordinateButton!=msg.buttons[0] && msg.buttons[0]==0 ) {
        coordinateButton = 0;
      }
    }

    // Compute Quad Control
    if ( quadIndex==0 ) {   // quad 1
      publishQuadCommand( 0, waypoint_quad1_control_pub_, x_1, y_1, y_1_, R_wr1 );
    }
    else {                  // quad 2
      publishQuadCommand( 1, waypoint_quad2_control_pub_, x_2, y_2, y_2_, R_wr2 );
    }

    if ( displayData ) {
      cout << "Manual Control: " << endl << manualControl << endl;
      cout << "Manual Yaw Control: " << endl << yawControlToggle << endl;
      cout << "Quad Control Index: " << endl << quadToggle << endl;
      cout << "Relative Coordinate Control: " << endl << relativeToggle << endl;
      cout << "joyStick Input: " << endl << joyInput << endl;
      cout << "----------- ----------- -----------" << endl;
    }

  }

  void quad1Callback(const geometry_msgs::Pose& msg) {

    // Extract State
    quad1PoseMsg = msg;
    x_1(0) = msg.position.x;
    x_1(1) = msg.position.y;
    x_1(2) = msg.position.z;
    double qx = msg.orientation.x;
    double qy = msg.orientation.y;
    double qz = msg.orientation.z;
    double qw = msg.orientation.w;

    // Conversion to Euler Angles
    double phiCur, theCur, psiCur;
    tf::Quaternion q(qx, qy, qz, qw);
    tf::Matrix3x3 m(q);
    m.getRPY(phiCur, theCur, psiCur);
    MatrixXd R_phi = MatrixXd::Zero(3,3);
    MatrixXd R_the = MatrixXd::Zero(3,3);
    MatrixXd R_psi = MatrixXd::Zero(3,3);
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
    R_wr1 = R_psi*R_the*R_phi;
    // Only Using Yaw Rotation Matrix
    R_wr1 = R_psi;

    x_1(3) = phiCur;
    x_1(4) = theCur;
    x_1(5) = psiCur;

    publishQuadCommand( 0, waypoint_quad1_control_pub_, x_1, y_1, y_1_, R_wr1 );

  }

  void quad2Callback(const geometry_msgs::Pose& msg) {

    // Extract State
    quad2PoseMsg = msg;
    x_2(0) = msg.position.x;
    x_2(1) = msg.position.y;
    x_2(2) = msg.position.z;
    double qx = msg.orientation.x;
    double qy = msg.orientation.y;
    double qz = msg.orientation.z;
    double qw = msg.orientation.w;

    // Conversion to Euler Angles
    double phiCur, theCur, psiCur;
    tf::Quaternion q(qx, qy, qz, qw);
    tf::Matrix3x3 m(q);
    m.getRPY(phiCur, theCur, psiCur);
    MatrixXd R_phi = MatrixXd::Zero(3,3);
    MatrixXd R_the = MatrixXd::Zero(3,3);
    MatrixXd R_psi = MatrixXd::Zero(3,3);
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
    R_wr2 = R_psi*R_the*R_phi;
    // Only Using Yaw Rotation Matrix
    R_wr2 = R_psi;

    x_2(3) = phiCur;
    x_2(4) = theCur;
    x_2(5) = psiCur;

    publishQuadCommand( 1, waypoint_quad2_control_pub_, x_2, y_2, y_2_, R_wr2 );

  }

  void measurement1Callback(const geometry_msgs::Twist& msg) {

    // Extract Data
    y_1(0) = msg.linear.x;
    y_1(1) = msg.linear.y;
    y_1(2) = msg.linear.z;

    publishQuadCommand( 0, waypoint_quad1_control_pub_, x_1, y_1, y_1_, R_wr1 );

  }

  void measurement2Callback(const geometry_msgs::Twist& msg) {

    // Extract Data
    y_2(0) = msg.linear.x;
    y_2(1) = msg.linear.y;
    y_2(2) = msg.linear.z;

    publishQuadCommand( 1, waypoint_quad2_control_pub_, x_2, y_2, y_2_, R_wr2 );

  }

};

int main(int argc, char **argv) {
  ros::init(argc, argv, "filter");
  ros::NodeHandle nh("~");

  int DISPLAY_DATA_FLAG;
  nh.param<int>("DISPLAY_DATA_FLAG", DISPLAY_DATA_FLAG, 0);
  int CONTROLLER_INDEX;
  nh.param<int>("CONTROLLER_INDEX", CONTROLLER_INDEX, 0);
  
  waypointQuadControlClass control(
    DISPLAY_DATA_FLAG, CONTROLLER_INDEX
  );
  
  while (ros::ok()) {
    ros::spinOnce();
  }
  ros::shutdown();
  
  return 0;
}

