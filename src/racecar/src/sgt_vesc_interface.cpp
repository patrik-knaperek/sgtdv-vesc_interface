/*****************************************************/
//Organization: Stuba Green Team
//Authors: Matej Dudák, Patrik Knaperek
/*****************************************************/

#include "../include/sgt_vesc_interface.h"
#include "geometry_msgs/Quaternion.h"
#include <tf/transform_datatypes.h>

SgtVescInterface::SgtVescInterface(ros::NodeHandle &handle)
{
    loadParams(handle);

    motor_speed_cmd_pub_ = handle.advertise<std_msgs::Float64>("vesc/commands/motor/speed", 1);
    servo_position_cmd_pub_ = handle.advertise<std_msgs::Float64>("vesc/commands/servo/position", 1);

    pathtracking_sub_ = handle.subscribe("pathtracking_commands", 1, &SgtVescInterface::controlCallback, this);

#ifdef VESC_ODOMETRY
    pose_estimate_pub_ = handle.advertise<sgtdv_msgs::CarPose>("pose_estimate",1);
    velocity_estimate_pub_ = handle.advertise<sgtdv_msgs::CarVel>("velocity_estimate",1);

    odom_sub_ = handle.subscribe("/vesc/odom", 1, &SgtVescInterface::odomCallback, this);

    sleep(1);   // wait for VESC to switch to MODE_OPERATING
    init();
#endif
}

void SgtVescInterface::loadParams(ros::NodeHandle &handle)
{
    getParam(handle, "sgt/cmd_speed_max", &params_.cmd_speed_in_max);
    getParam(handle, "sgt/cmd_speed_min", &params_.cmd_speed_in_min);
    getParam(handle, "vesc/vesc_driver/speed_max", &params_.cmd_speed_out_max);
    getParam(handle, "vesc/vesc_driver/speed_th", &params_.cmd_speed_out_min);
    getParam(handle, "sgt/cmd_steering_max", &params_.cmd_steer_in_max);
    getParam(handle, "sgt/cmd_steering_min", &params_.cmd_steer_in_min);
    getParam(handle, "vesc/vesc_driver/servo_max", &params_.cmd_steer_out_max);
    getParam(handle, "vesc/vesc_driver/servo_min", &params_.cmd_steer_out_min);
    getParam(handle, "vesc/steering_angle_to_servo_offset", &params_.cmd_steer_out_offset);
    getParam(handle, "vesc/steering_angle_to_servo_gain", &params_.cmd_steer_out_gain);
    
    params_.k = (params_.cmd_speed_out_max - params_.cmd_speed_out_min) / (params_.cmd_speed_in_max - params_.cmd_speed_in_min);
    params_.q = params_.cmd_speed_out_min - params_.k * params_.cmd_speed_in_min;
}

template<typename T>
void SgtVescInterface::getParam(const ros::NodeHandle &handle, const std::string &name, T* storage) const
{
    if (!handle.getParam(name, *storage))
        ROS_ERROR("Failed to get parameter \"%s\" from server\n", name.data());
}

template<typename T> 
void SgtVescInterface::getParam(const ros::NodeHandle &handle, const std::string &name,
                            const T &default_value, T* storage) const
{
    if (!handle.param<T>(name, *storage, default_value))
        ROS_WARN_STREAM("Failed to get parameter " << name.data() << " from server, setting default: " << default_value);
}


void SgtVescInterface::controlCallback(const sgtdv_msgs::Control::ConstPtr &control_msg) {
    std_msgs::Float64 motor_speed_msg, servo_position_msg;
    
    // speed command
    if (control_msg->speed == 0.0)
    {
	    motor_speed_msg.data = control_msg->speed;
    }
    else
    {
    	motor_speed_msg.data = control_msg->speed * params_.k + params_.q;
    	if (motor_speed_msg.data > params_.cmd_speed_out_max)
            motor_speed_msg.data = params_.cmd_speed_out_max;
    }
    motor_speed_cmd_pub_.publish(motor_speed_msg);

    // steering command
    servo_position_msg.data = params_.cmd_steer_out_gain * control_msg->steeringAngle + params_.cmd_steer_out_offset;
    if (servo_position_msg.data > params_.cmd_steer_out_max)
        servo_position_msg.data = params_.cmd_steer_out_max;
    else if (servo_position_msg.data < params_.cmd_steer_out_min)
        servo_position_msg.data = params_.cmd_steer_out_min;
    servo_position_cmd_pub_.publish(servo_position_msg);
}

#ifdef VESC_ODOMETRY
    // vesc_to_odom_node won't start publishing /odom topic before it receives servo command
    void SgtVescInterface::init()
    {
        std_msgs::Float64 servo_position;
        servo_position.data = params_.cmd_steer_out_offset;
        servo_position_cmd_pub_.publish(servo_position);
    }
    
    void SgtVescInterface::odomCallback(const nav_msgs::Odometry &odom_msg)
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
        car_vel.yawRate = odom_msg.twist.twist.angular.z;

//        pose_estimate_pub_.publish(car_pose);
        velocity_estimate_pub_.publish(car_vel);
    }
#endif // VESC_ODOMETRY
