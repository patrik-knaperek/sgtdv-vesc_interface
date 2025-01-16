/*****************************************************/
//Organization: Stuba Green Team
//Authors: Matej Dudák, Patrik Knaperek
/*****************************************************/

#pragma once

// ROS
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/Joy.h>
#include <std_srvs/Empty.h>

// SGT
#include <sgtdv_msgs/Control.h>
#include <sgtdv_msgs/CarPose.h>
#include <sgtdv_msgs/CarVel.h>

// #define VESC_ODOMETRY

class VescInterface
{
public:
  VescInterface(ros::NodeHandle &handle);
  ~VescInterface() = default;

  struct Params
  {
    float sgt_cmd_speed_max;
    float sgt_cmd_speed_min;
    float joy_cmd_speed_max;
    float joy_cmd_speed_min;
    float vesc_cmd_speed_max;
    float vesc_cmd_speed_th;
    float vesc_cmd_speed_min;
    float sgt_cmd_steer_max;
    float sgt_cmd_steer_min;
    float joy_cmd_steer_max;
    float joy_cmd_steer_min;
    float vesc_cmd_steer_max;
    float vesc_cmd_steer_min;
    float vesc_cmd_steer_offset;
    float vesc_cmd_steer_gain;
    float sgt_vesc_k;
    float sgt_vesc_q;
    float joy_vesc_k1, joy_vesc_k2;
    float joy_vesc_q1, joy_vesc_q2;
    int speed_axis;
    int steering_axis;
    int deadman_button;
    int start_button;
  };

  void sgtCmdCallback(const sgtdv_msgs::Control::ConstPtr &control_msg);
  void joyCmdCallback(const sensor_msgs::Joy::ConstPtr &control_msg);
  void resetOdomCallback(const std_msgs::Empty::ConstPtr &msg);
  void init();

#ifdef VESC_ODOMETRY
  void setPublisherPose(const ros::Publisher &pose_pub);
  void setPublisherVelocity(const ros::Publisher &vel_pub);
  void odomCallback(const nav_msgs::Odometry &odom_msg);
#endif

private:
  void loadParams(ros::NodeHandle &handle);
  template<typename T> void getParam(const ros::NodeHandle &handle, const std::string &name, T* storage) const;
  template<typename T> void getParam(const ros::NodeHandle &handle, const std::string &name,
                                      const T &default_value, T* storage) const;
  ros::Publisher motor_speed_cmd_pub_, servo_position_cmd_pub_, reset_odom_pub_;
  ros::Subscriber pathtracking_sub_, joy_sub_, reset_odom_sub_;
  std_srvs::Empty stop_msg_, start_msg_;

  bool deadman_switch_ = false;
  
  Params params_;

#ifdef VESC_ODOMETRY
  ros::Publisher pose_estimate_pub_, velocity_estimate_pub_;
  ros::Subscriber odom_sub_;
#endif
};
