#ifndef MAVROS_POSE_CONTROL_H_
#define MAVROS_POSE_CONTROL_H_

#include <mavros_pose_control/PID.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Empty.h>
#include <tf/transform_datatypes.h>
#include <dynamic_reconfigure/server.h>
#include <mavros_pose_control/ControlTunerConfig.h>

#include <thread>
#include <mutex>
#include <boost/bind.hpp>

class mavrosPoseController {
 public:
   mavrosPoseController();
   ~mavrosPoseController();

 private:
   // PID controllers
   std::shared_ptr<PIDController> z_pos_; // pid controller for z pos -> z vel
   std::shared_ptr<PIDController> z_vel_; // pid controller for z vel -> z cmd

   std::shared_ptr<PIDController> y_pos_; // pid controller for y pos -> y vel
   std::shared_ptr<PIDController> y_vel_; // pid controller for y vel -> y cmd

   std::shared_ptr<PIDController> x_pos_; // pid controller for x pos -> x vel
   std::shared_ptr<PIDController> x_vel_; // pid controller for x vel -> x cmd

   double z_throttle_zero_;
   double z_throttle_divider_;

   // callback and data handlers for getting the pose from whatever source
   void feedbackPoseCb( geometry_msgs::PoseStamped msg );
   std::mutex feedback_data_lock_;
   void setpointPoseCb( geometry_msgs::Pose msg );
   std::mutex setpoint_data_lock_;
   void integralResetCb( std_msgs::Empty msg );

   std::shared_ptr<geometry_msgs::Pose> desired_set_point_;
   std::shared_ptr<geometry_msgs::PoseStamped> cur_pose_;
   std::shared_ptr<geometry_msgs::TwistStamped> cur_twist_;

   double rate_ = 100.0;
   std::mutex alive_;
   std::thread control_thread_;
   void control_loop();

   ros::Publisher cpPub; // PoseStamped
   ros::Publisher ctPub; // TwistStamped
   ros::Publisher attThrottlePub; // Float64
   ros::Publisher attRollPub; // Float64
   ros::Publisher attPitchPub; // Float64
   ros::Publisher attitudePub; // Twist 

   ros::Subscriber feedbackPoseSub; // PoseStamped
   ros::Subscriber setpointPoseSub; // PoseStamped
   ros::Subscriber resetSub; // Empty

   dynamic_reconfigure::Server<mavros_pose_control::ControlTunerConfig> server;
   dynamic_reconfigure::Server<mavros_pose_control::ControlTunerConfig>::CallbackType f;
   void configCb( mavros_pose_control::ControlTunerConfig &config, uint32_t level );
};

#endif
