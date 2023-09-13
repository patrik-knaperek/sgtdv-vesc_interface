/*****************************************************/
//Organization: Stuba Green Team
//Authors: Matej Dudák, Patrik Knaperek
/*****************************************************/

#include "../include/vesc_interface.h"
#include "geometry_msgs/Quaternion.h"
#include <tf/transform_datatypes.h>

VescInterface::VescInterface(ros::NodeHandle &handle)
{
    loadParams(handle);

    motor_speed_cmd_pub_ = handle.advertise<std_msgs::Float64>("vesc/commands/motor/speed", 1);
    servo_position_cmd_pub_ = handle.advertise<std_msgs::Float64>("vesc/commands/servo/position", 1);

    pathtracking_sub_ = handle.subscribe("pathtracking_commands", 1, &VescInterface::sgtCmdCallback, this);
    joy_sub_ = handle.subscribe("joy", 1, &VescInterface::joyCmdCallback, this);

#ifdef VESC_ODOMETRY
    pose_estimate_pub_ = handle.advertise<sgtdv_msgs::CarPose>("pose_estimate",1);
    velocity_estimate_pub_ = handle.advertise<sgtdv_msgs::CarVel>("velocity_estimate",1);

    odom_sub_ = handle.subscribe("/vesc/odom", 1, &SgtVescInterface::odomCallback, this);

    sleep(1);   // wait for VESC to switch to MODE_OPERATING
    init();
#endif
}

void VescInterface::loadParams(ros::NodeHandle &handle)
{
    getParam(handle, "sgt/cmd_speed_max", &params_.sgt_cmd_speed_max);
    getParam(handle, "sgt/cmd_speed_min", &params_.sgt_cmd_speed_min);
    getParam(handle, "vesc/vesc_driver/speed_max", &params_.vesc_cmd_speed_max);
    getParam(handle, "vesc/vesc_driver/speed_th", &params_.vesc_cmd_speed_th);
    getParam(handle, "vesc/vesc_driver/speed_min", &params_.vesc_cmd_speed_min);
    getParam(handle, "sgt/cmd_steering_max", &params_.sgt_cmd_steer_max);
    getParam(handle, "sgt/cmd_steering_min", &params_.sgt_cmd_steer_min);

    getParam(handle, "joy/cmd_speed_max", &params_.joy_cmd_speed_max);
    getParam(handle, "joy/cmd_speed_min", &params_.joy_cmd_speed_min);
    getParam(handle, "joy/cmd_steering_max", &params_.joy_cmd_steer_max);
    getParam(handle, "joy/cmd_steering_min", &params_.joy_cmd_steer_min);
    getParam(handle, "joy/axis_speed", &params_.speed_axis);
    getParam(handle, "joy/axis_steering", &params_.steering_axis);
    getParam(handle, "joy/deadman_button", &params_.deadman_button);
    getParam(handle, "joy/start_button", &params_.start_button);
    
    getParam(handle, "vesc/vesc_driver/servo_max", &params_.vesc_cmd_steer_max);
    getParam(handle, "vesc/vesc_driver/servo_min", &params_.vesc_cmd_steer_min);
    getParam(handle, "vesc/steering_angle_to_servo_offset", &params_.vesc_cmd_steer_offset);
    getParam(handle, "vesc/steering_angle_to_servo_gain", &params_.vesc_cmd_steer_gain);
    
    params_.sgt_vesc_k = (params_.vesc_cmd_speed_max - params_.vesc_cmd_speed_th) 
                        / (params_.sgt_cmd_speed_max - params_.sgt_cmd_speed_min);
    params_.sgt_vesc_q = params_.vesc_cmd_speed_th - params_.sgt_vesc_k * params_.sgt_cmd_speed_min;

    params_.joy_vesc_k1 = params_.vesc_cmd_speed_max / params_.joy_cmd_speed_max;
    params_.joy_vesc_q1 = params_.vesc_cmd_speed_max - params_.joy_vesc_k1 * params_.joy_cmd_speed_max;

    params_.joy_vesc_k2 = params_.vesc_cmd_speed_min / params_.joy_cmd_speed_min;
    params_.joy_vesc_q2 = params_.vesc_cmd_speed_min - params_.joy_vesc_k2 * params_.joy_cmd_speed_min;
}

template<typename T>
void VescInterface::getParam(const ros::NodeHandle &handle, const std::string &name, T* storage) const
{
    if (!handle.getParam(name, *storage))
        ROS_ERROR("Failed to get parameter \"%s\" from server\n", name.data());
}

template<typename T> 
void VescInterface::getParam(const ros::NodeHandle &handle, const std::string &name,
                            const T &default_value, T* storage) const
{
    if (!handle.param<T>(name, *storage, default_value))
        ROS_WARN_STREAM("Failed to get parameter " << name.data() << " from server, setting default: " << default_value);
}


void VescInterface::sgtCmdCallback(const sgtdv_msgs::Control::ConstPtr &control_msg)
{
    if (deadman_switch_) return;
    
    std_msgs::Float64 motor_speed_msg, servo_position_msg;
    
    /* speed command */
    if (control_msg->speed == 0.0)
    {
	    motor_speed_msg.data = control_msg->speed;
    }
    else
    {
    	motor_speed_msg.data = control_msg->speed * params_.sgt_vesc_k + params_.sgt_vesc_q;
    	if (motor_speed_msg.data > params_.vesc_cmd_speed_max)
            motor_speed_msg.data = params_.vesc_cmd_speed_max;
    }
    motor_speed_cmd_pub_.publish(motor_speed_msg);

    /* steering command */
    servo_position_msg.data = params_.vesc_cmd_steer_gain * control_msg->steeringAngle + params_.vesc_cmd_steer_offset;
    if (servo_position_msg.data > params_.vesc_cmd_steer_max)
        servo_position_msg.data = params_.vesc_cmd_steer_max;
    else if (servo_position_msg.data < params_.vesc_cmd_steer_min)
        servo_position_msg.data = params_.vesc_cmd_steer_min;
    servo_position_cmd_pub_.publish(servo_position_msg);
}

void VescInterface::joyCmdCallback(const sensor_msgs::Joy::ConstPtr &control_msg)
{
    std_msgs::Float64 motor_speed_msg, servo_position_msg;
    
    if (control_msg->buttons[params_.start_button])
    {
	    if (!ros::service::call("pathTracking/start", start_msg_))
            {
                ROS_WARN("Service \"pathTracking/start\" failed");
            }
    }

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

        /* speed command */
        if (control_msg->axes[params_.speed_axis] == 0.0)
        {
            motor_speed_msg.data = control_msg->axes[params_.speed_axis];
        }
        else if (control_msg->axes[params_.speed_axis] > 0)
        {
            motor_speed_msg.data = control_msg->axes[params_.speed_axis] * params_.joy_vesc_k1 + params_.joy_vesc_q1;
            if (motor_speed_msg.data > params_.vesc_cmd_speed_max)
                motor_speed_msg.data = params_.vesc_cmd_speed_max;
        }
        else
        {
            motor_speed_msg.data = control_msg->axes[params_.speed_axis] * params_.joy_vesc_k2 + params_.joy_vesc_q2;
            if (motor_speed_msg.data < params_.vesc_cmd_speed_min)
                motor_speed_msg.data = params_.vesc_cmd_speed_min;
        }
        motor_speed_cmd_pub_.publish(motor_speed_msg);

        /* steering command */
        servo_position_msg.data = params_.vesc_cmd_steer_gain * control_msg->axes[params_.steering_axis] + params_.vesc_cmd_steer_offset;
        if (servo_position_msg.data > params_.vesc_cmd_steer_max)
            servo_position_msg.data = params_.vesc_cmd_steer_max;
        else if (servo_position_msg.data < params_.vesc_cmd_steer_min)
            servo_position_msg.data = params_.vesc_cmd_steer_min;
        servo_position_cmd_pub_.publish(servo_position_msg);
    }
    else if (deadman_switch_)
        deadman_switch_ = false;
    
}

#ifdef VESC_ODOMETRY
    /* vesc_to_odom_node won't start publishing /odom topic before it receives servo command */
    void VescInterface::init()
    {
        std_msgs::Float64 servo_position;
        servo_position.data = params_.vesc_cmd_steer_offset;
        servo_position_cmd_pub_.publish(servo_position);
    }
    
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
        car_vel.yawRate = odom_msg.twist.twist.angular.z;

//        pose_estimate_pub_.publish(car_pose);
        velocity_estimate_pub_.publish(car_vel);
    }
#endif // VESC_ODOMETRY
