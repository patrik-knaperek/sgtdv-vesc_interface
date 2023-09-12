/*****************************************************/
//Organization: Stuba Green Team
//Authors: Patrik Knaperek
/*****************************************************/

#include "../include/joy_vesc_interface.h"
#include "geometry_msgs/Quaternion.h"
#include <tf/transform_datatypes.h>

JoyVescInterface::JoyVescInterface(ros::NodeHandle &handle)
{
    loadParams(handle);

    motor_speed_cmd_pub_ = handle.advertise<std_msgs::Float64>("vesc/commands/motor/speed", 1);
    servo_position_cmd_pub_ = handle.advertise<std_msgs::Float64>("vesc/commands/servo/position", 1);

    joy_sub_ = handle.subscribe("joy", 1, &JoyVescInterface::controlCallback, this);
}

void JoyVescInterface::loadParams(ros::NodeHandle &handle)
{
    getParam(handle, "joy_node/cmd_speed_max", &params_.cmd_speed_in_max);
    getParam(handle, "joy_node/cmd_speed_min", &params_.cmd_speed_in_min);
    getParam(handle, "joy_node/cmd_steering_max", &params_.cmd_steer_in_max);
    getParam(handle, "joy_node/cmd_steering_min", &params_.cmd_steer_in_min);
    getParam(handle, "joy_node/axis_speed", &params_.speed_axis);
    getParam(handle, "joy_node/axis_steering", &params_.steering_axis);
    getParam(handle, "joy_node/deadman_button", &params_.deadman_button);
    getParam(handle, "vesc/vesc_driver/speed_max", &params_.cmd_speed_out_max);
    getParam(handle, "vesc/vesc_driver/speed_min", &params_.cmd_speed_out_min);
    getParam(handle, "vesc/vesc_driver/servo_max", &params_.cmd_steer_out_max);
    getParam(handle, "vesc/vesc_driver/servo_min", &params_.cmd_steer_out_min);
    getParam(handle, "vesc/steering_angle_to_servo_offset", &params_.cmd_steer_out_offset);
    getParam(handle, "vesc/steering_angle_to_servo_gain", &params_.cmd_steer_out_gain);
    
    params_.k1 = params_.cmd_speed_out_max / params_.cmd_speed_in_max;
    params_.q1 = params_.cmd_speed_out_max - params_.k1 * params_.cmd_speed_in_max;

    params_.k2 = params_.cmd_speed_out_min / params_.cmd_speed_in_min;
    params_.q2 = params_.cmd_speed_out_min - params_.k2 * params_.cmd_speed_in_min;
}

template<typename T>
void JoyVescInterface::getParam(const ros::NodeHandle &handle, const std::string &name, T* storage)
{
    if (!handle.getParam(name, *storage))
        ROS_ERROR("Failed to get parameter \"%s\" from server\n", name.data());
}

template<typename T> 
void JoyVescInterface::getParam(const ros::NodeHandle &handle, const std::string &name,
                            const T &default_value, T* storage)
{
    if (!handle.param<T>(name, *storage, default_value))
        ROS_WARN_STREAM("Failed to get parameter " << name.data() << " from server, setting default: " << default_value);
}


void JoyVescInterface::controlCallback(const sensor_msgs::Joy::ConstPtr &control_msg) {
    std_msgs::Float64 motor_speed_msg, servo_position_msg;
    
    if (control_msg->buttons[params_.deadman_button])
    {
        if (!deadman_switch_)
        {
            if (!ros::service::call("pathTracking/stop", stop_msg_))
            {
                ROS_WARN("Service \"pathTracking/stop\" failed");
            }
            deadman_switch_ = true;
        }

        // speed command
        if (control_msg->axes[params_.speed_axis] == 0.0)
        {
            motor_speed_msg.data = control_msg->axes[params_.speed_axis];
        }
        else if (control_msg->axes[params_.speed_axis] > 0)
        {
            motor_speed_msg.data = control_msg->axes[params_.speed_axis] * params_.k1 + params_.q1;
            if (motor_speed_msg.data > params_.cmd_speed_out_max)
                motor_speed_msg.data = params_.cmd_speed_out_max;
        }
        else
        {
            motor_speed_msg.data = control_msg->axes[params_.speed_axis] * params_.k2 + params_.q2;
            if (motor_speed_msg.data < params_.cmd_speed_out_min)
                motor_speed_msg.data = params_.cmd_speed_out_min;
        }
        motor_speed_cmd_pub_.publish(motor_speed_msg);

        // steering command
        servo_position_msg.data = params_.cmd_steer_out_gain * control_msg->axes[params_.steering_axis] + params_.cmd_steer_out_offset;
        if (servo_position_msg.data > params_.cmd_steer_out_max)
            servo_position_msg.data = params_.cmd_steer_out_max;
        else if (servo_position_msg.data < params_.cmd_steer_out_min)
            servo_position_msg.data = params_.cmd_steer_out_min;
        servo_position_cmd_pub_.publish(servo_position_msg);
    }
    else if (deadman_switch_)
        deadman_switch_ = false;
    
}