/*****************************************************/
//Organization: Stuba Green Team
//Authors: Matej Dudák, Patrik Knaperek
/*****************************************************/

/* ROS */
#include <std_msgs/Float64.h>
#include "geometry_msgs/Quaternion.h"
#include <tf/transform_datatypes.h>

/* SGT-DV */
#include <sgtdv_msgs/CarPose.h>
#include <sgtdv_msgs/CarVel.h>
#include "SGT_Utils.h"

/* Header */
#include "../include/vesc_interface.h"

VescInterface::VescInterface(ros::NodeHandle &handle)
  /* ROS interface initialization */
  : motor_speed_cmd_pub_(handle.advertise<std_msgs::Float64>("vesc/commands/motor/speed", 1))
  , servo_position_cmd_pub_(handle.advertise<std_msgs::Float64>("vesc/commands/servo/position", 1))
  , reset_odom_pub_(handle.advertise<std_msgs::Empty>("vesc/vesc_to_odom/reset_odometry", 1))

  , pathtracking_sub_(handle.subscribe("path_tracking/cmd", 1, &VescInterface::sgtCmdCallback, this))
  , joy_sub_(handle.subscribe("joy", 1, &VescInterface::joyCmdCallback, this))
  , reset_odom_sub_(handle.subscribe("reset_odometry", 1, &VescInterface::resetOdomCallback, this))

#ifdef VESC_ODOMETRY
  , pose_estimate_pub_(handle.advertise<sgtdv_msgs::CarPose>("odometry/pose",1))
  , velocity_estimate_pub_(handle.advertise<sgtdv_msgs::CarVel>("odometry/velocity",1))

  , odom_sub_(handle.subscribe("/vesc/odom", 1, &VescInterface::odomCallback, this))
#endif
{
  loadParams(handle);

  sleep(2);   // wait for VESC to switch to MODE_OPERATING
  init();
}

void VescInterface::loadParams(const ros::NodeHandle &handle)
{
  Utils::loadParam(handle, "sgt/cmd_speed_max", &params_.sgt_cmd_speed_max);
  Utils::loadParam(handle, "sgt/cmd_speed_min", &params_.sgt_cmd_speed_min);
  Utils::loadParam(handle, "vesc/vesc_driver/speed_max", &params_.vesc_cmd_speed_max);
  Utils::loadParam(handle, "vesc/vesc_driver/speed_th", &params_.vesc_cmd_speed_th);
  Utils::loadParam(handle, "vesc/vesc_driver/speed_min", &params_.vesc_cmd_speed_min);
  Utils::loadParam(handle, "sgt/cmd_steering_max", &params_.sgt_cmd_steer_max);
  Utils::loadParam(handle, "sgt/cmd_steering_min", &params_.sgt_cmd_steer_min);

  Utils::loadParam(handle, "joy/cmd_speed_max", &params_.joy_cmd_speed_max);
  Utils::loadParam(handle, "joy/cmd_speed_min", &params_.joy_cmd_speed_min);
  Utils::loadParam(handle, "joy/cmd_steering_max", &params_.joy_cmd_steer_max);
  Utils::loadParam(handle, "joy/cmd_steering_min", &params_.joy_cmd_steer_min);
  Utils::loadParam(handle, "joy/axis_speed", &params_.speed_axis);
  Utils::loadParam(handle, "joy/axis_steering", &params_.steering_axis);
  Utils::loadParam(handle, "joy/deadman_button", &params_.deadman_button);
  Utils::loadParam(handle, "joy/start_button", &params_.start_button);
  
  Utils::loadParam(handle, "vesc/vesc_driver/servo_max", &params_.vesc_cmd_steer_max);
  Utils::loadParam(handle, "vesc/vesc_driver/servo_min", &params_.vesc_cmd_steer_min);
  Utils::loadParam(handle, "vesc/steering_angle_to_servo_offset", &params_.vesc_cmd_steer_offset);
  Utils::loadParam(handle, "vesc/steering_angle_to_servo_gain", &params_.vesc_cmd_steer_gain);
  
  params_.sgt_vesc_k = (params_.vesc_cmd_speed_max - params_.vesc_cmd_speed_th) 
                      / (params_.sgt_cmd_speed_max - params_.sgt_cmd_speed_min);
  params_.sgt_vesc_q = params_.vesc_cmd_speed_th - params_.sgt_vesc_k * params_.sgt_cmd_speed_min;

  params_.joy_vesc_k1 = params_.vesc_cmd_speed_max / params_.joy_cmd_speed_max;
  params_.joy_vesc_q1 = params_.vesc_cmd_speed_max - params_.joy_vesc_k1 * params_.joy_cmd_speed_max;

  params_.joy_vesc_k2 = params_.vesc_cmd_speed_min / params_.joy_cmd_speed_min;
  params_.joy_vesc_q2 = params_.vesc_cmd_speed_min - params_.joy_vesc_k2 * params_.joy_cmd_speed_min;
}

void VescInterface::sgtCmdCallback(const sgtdv_msgs::Control::ConstPtr &control_msg)
{
  if(deadman_switch_) return;
  
  std_msgs::Float64 motor_speed_msg, servo_position_msg;
  
  /* speed command */
  if(control_msg->speed == 0.0)
  {
    motor_speed_msg.data = control_msg->speed;
  }
  else
  {
    motor_speed_msg.data = control_msg->speed * params_.sgt_vesc_k + params_.sgt_vesc_q;
    if(motor_speed_msg.data > params_.vesc_cmd_speed_max)
      motor_speed_msg.data = params_.vesc_cmd_speed_max;
  }
  motor_speed_cmd_pub_.publish(motor_speed_msg);

  /* steering command */
  servo_position_msg.data = params_.vesc_cmd_steer_gain * control_msg->steering_angle + params_.vesc_cmd_steer_offset;
  if(servo_position_msg.data > params_.vesc_cmd_steer_max)
    servo_position_msg.data = params_.vesc_cmd_steer_max;
  else if(servo_position_msg.data < params_.vesc_cmd_steer_min)
    servo_position_msg.data = params_.vesc_cmd_steer_min;
  servo_position_cmd_pub_.publish(servo_position_msg);
}

void VescInterface::joyCmdCallback(const sensor_msgs::Joy::ConstPtr &control_msg)
{
  std_msgs::Float64 motor_speed_msg, servo_position_msg;
  
  if(control_msg->buttons[params_.start_button])
  {
    if(!ros::service::call("path_tracking/start", start_msg_))
    {
      ROS_WARN("Service \"path_tracking/start\" failed");
    }
  }

  if(control_msg->buttons[params_.deadman_button])
  {
    if(!deadman_switch_)
    {
      if(!ros::service::call("path_tracking/stop", stop_msg_))
      {
        ROS_WARN("Service \"path_tracking/stop\" failed");
      }
      deadman_switch_ = true;
    }

    /* speed command */
    if(control_msg->axes[params_.speed_axis] == 0.0)
    {
      motor_speed_msg.data = control_msg->axes[params_.speed_axis];
    }
    else if(control_msg->axes[params_.speed_axis] > 0)
    {
      motor_speed_msg.data = control_msg->axes[params_.speed_axis] * params_.joy_vesc_k1 + params_.joy_vesc_q1;
      if(motor_speed_msg.data > params_.vesc_cmd_speed_max)
        motor_speed_msg.data = params_.vesc_cmd_speed_max;
    }
    else
    {
      motor_speed_msg.data = control_msg->axes[params_.speed_axis] * params_.joy_vesc_k2 + params_.joy_vesc_q2;
      if(motor_speed_msg.data < params_.vesc_cmd_speed_min)
      motor_speed_msg.data = params_.vesc_cmd_speed_min;
    }
    motor_speed_cmd_pub_.publish(motor_speed_msg);

    /* steering command */
    servo_position_msg.data = params_.vesc_cmd_steer_gain * control_msg->axes[params_.steering_axis] + params_.vesc_cmd_steer_offset;
    if(servo_position_msg.data > params_.vesc_cmd_steer_max)
      servo_position_msg.data = params_.vesc_cmd_steer_max;
    else if(servo_position_msg.data < params_.vesc_cmd_steer_min)
      servo_position_msg.data = params_.vesc_cmd_steer_min;
    servo_position_cmd_pub_.publish(servo_position_msg);
  }
  else if(deadman_switch_)
    deadman_switch_ = false;
  
}

void VescInterface::resetOdomCallback(const std_msgs::Empty::ConstPtr &msg)
{
  reset_odom_pub_.publish(std_msgs::Empty());
}

/* vesc_to_odom_node won't start publishing /odom topic before it receives servo command */
void VescInterface::init()
{
  std_msgs::Float64 servo_position;
  servo_position.data = params_.vesc_cmd_steer_offset;
  servo_position_cmd_pub_.publish(servo_position);
}
    
#ifdef VESC_ODOMETRY
void VescInterface::odomCallback(const nav_msgs::Odometry &odom_msg)
{
  sgtdv_msgs::CarPose car_pose;
  car_pose.position.x = odom_msg.pose.pose.position.x;
  car_pose.position.y = odom_msg.pose.pose.position.y;
  const auto z = odom_msg.pose.pose.orientation.z;
  tf::Quaternion orientation_q;
  tf::quaternionMsgToTF(odom_msg.pose.pose.orientation, orientation_q);
  {
    static double roll = 0.0, pitch = 0.0, yaw = 0.0;
    tf::Matrix3x3(orientation_q).getRPY(roll, pitch, yaw);
    car_pose.yaw = yaw;
  }
  car_pose.covariance = odom_msg.pose.covariance;

  sgtdv_msgs::CarVel car_vel;
  car_vel.speed = odom_msg.twist.twist.linear.x;
  car_vel.yaw_rate = odom_msg.twist.twist.angular.z;

// pose_estimate_pub_.publish(car_pose);
  velocity_estimate_pub_.publish(car_vel);
}
#endif // VESC_ODOMETRY
