/*****************************************************/
//Organization: Stuba Green Team
//Authors: Matej Dudák, Patrik Knaperek
/*****************************************************/

#pragma once

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <sgtdv_msgs/Control.h>
#include <sgtdv_msgs/CarPose.h>
#include <sgtdv_msgs/CarVel.h>

#define VESC_ODOMETRY

class SgtVescInterface {
public:
    SgtVescInterface(ros::NodeHandle &handle);
    ~SgtVescInterface() = default;

    struct Params
    {
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
        float k;
        float q;
    };

    void controlCallback(const sgtdv_msgs::Control::ConstPtr &control_msg);
#ifdef VESC_ODOMETRY
    void init();
    void setPublisherPose(const ros::Publisher &pose_pub);
    void setPublisherVelocity(const ros::Publisher &vel_pub);
    void odomCallback(const nav_msgs::Odometry &odom_msg);
#endif

private:
    void loadParams(ros::NodeHandle &handle);
    template<typename T> void getParam(const ros::NodeHandle &handle, const std::string &name, T* storage) const;
    template<typename T> void getParam(const ros::NodeHandle &handle, const std::string &name,
                                        const T &default_value, T* storage) const;
    ros::Publisher motor_speed_cmd_pub_, servo_position_cmd_pub_;
    ros::Subscriber pathtracking_sub_;
    
    Params params_;

#ifdef VESC_ODOMETRY
    ros::Publisher pose_estimate_pub_, velocity_estimate_pub_;
    ros::Subscriber odom_sub_;
#endif
};
