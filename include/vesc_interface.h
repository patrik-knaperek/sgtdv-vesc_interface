/*****************************************************/
//Organization: Stuba Green Team
//Authors: Matej Dudák, Patrik Knaperek
/*****************************************************/

#pragma once

/* ROS */
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/Joy.h>
#include <std_srvs/Empty.h>

/* SGT-DV */
#include <sgtdv_msgs/Control.h>

#define VESC_ODOMETRY

class VescInterface
{
public:
  VescInterface(ros::NodeHandle &handle);
  ~VescInterface() = default;

  void sgtCmdCallback(const sgtdv_msgs::Control::ConstPtr &control_msg);
  void joyCmdCallback(const sensor_msgs::Joy::ConstPtr &control_msg);
  void resetOdomCallback(const std_msgs::Empty::ConstPtr &msg);
  void init();

#ifdef VESC_ODOMETRY
  void odomCallback(const nav_msgs::Odometry &odom_msg);
#endif

private:
  void loadParams(const ros::NodeHandle &handle);

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
  } params_;

  ros::Publisher motor_speed_cmd_pub_, servo_position_cmd_pub_, reset_odom_pub_;
  ros::Subscriber pathtracking_sub_, joy_sub_, reset_odom_sub_;
  std_srvs::Empty stop_msg_, start_msg_;

  bool deadman_switch_ = false;
  
#ifdef VESC_ODOMETRY
  ros::Publisher pose_estimate_pub_, velocity_estimate_pub_;
  ros::Subscriber odom_sub_;
#endif
};
