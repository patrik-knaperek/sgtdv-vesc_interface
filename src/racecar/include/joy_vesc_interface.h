/*****************************************************/
//Organization: Stuba Green Team
//Authors: Patrik Knaperek
/*****************************************************/

#pragma once

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Joy.h>
#include <std_srvs/Empty.h>

class JoyVescInterface {
public:
    JoyVescInterface(ros::NodeHandle &handle);
    ~JoyVescInterface() = default;


    struct Params
    {
        int speed_axis;
        int steering_axis;
        int deadman_button;
        float cmd_speed_in_max;
        float cmd_speed_in_min;
        float cmd_speed_out_max;
        float cmd_speed_out_min;
        float cmd_steer_in_max;
        float cmd_steer_in_min;
        float cmd_steer_out_max;
        float cmd_steer_out_min;
        float cmd_steer_out_offset;
        float cmd_steer_out_gain;
        float k1;
        float k2;
        float q1;
        float q2;
    };

    void controlCallback(const sensor_msgs::Joy::ConstPtr &control_msg);

private:
    void loadParams(ros::NodeHandle &handle);
    template<typename T> void getParam(const ros::NodeHandle &handle, const std::string &name, T* storage);
    template<typename T> void getParam(const ros::NodeHandle &handle, const std::string &name,
                                        const T &default_value, T* storage);
    ros::Publisher motor_speed_cmd_pub_, servo_position_cmd_pub_;
    ros::Subscriber joy_sub_;
    std_srvs::Empty stop_msg_;

    bool deadman_switch_ = false;
    
    Params params_;
};
