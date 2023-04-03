/*****************************************************/
//Organization: Stuba Green Team
//Authors: Matej Dudák, Patrik Knaperek
/*****************************************************/

#include "../include/RaceCar.h"
#include "geometry_msgs/Quaternion.h"
#include <tf/transform_datatypes.h>

RaceCar::RaceCar(ros::NodeHandle &handle)
{
    LoadParams(handle);
}

RaceCar::~RaceCar() = default;

void RaceCar::LoadParams(ros::NodeHandle &handle)
{
    getParam(handle, "sgt/cmd_speed_max", &m_params.cmdSpeedInMax);
    getParam(handle, "sgt/cmd_speed_min", &m_params.cmdSpeedInMin);
    getParam(handle, "vesc/vesc_driver/speed_max", &m_params.cmdSpeedOutMax);
    getParam(handle, "vesc/vesc_driver/speed_min", &m_params.cmdSpeedOutMin);
    getParam(handle, "sgt/cmd_steering_max", &m_params.cmdSteerInMax);
    getParam(handle, "sgt/cmd_steering_min", &m_params.cmdSteerInMin);
    getParam(handle, "vesc/vesc_driver/servo_max", &m_params.cmdSteerOutMax);
    getParam(handle, "vesc/vesc_driver/servo_min", &m_params.cmdSteerOutMin);
    getParam(handle, "vesc/steering_angle_to_servo_offset", &m_params.cmdSteerOutOffset);
    getParam(handle, "vesc/steering_angle_to_servo_gain", &m_params.cmdSteerOutGain);
    if (m_params.cmdSpeedInMin == 0)
    {
        m_params.k = m_params.cmdSpeedOutMax / m_params.cmdSpeedInMax;
        m_params.q = 0.0;
    } else
    {
        m_params.k = (m_params.cmdSpeedOutMax - m_params.cmdSpeedOutMin) / (m_params.cmdSpeedInMax - m_params.cmdSpeedInMin);
        m_params.q = m_params.cmdSpeedOutMin - m_params.k * m_params.cmdSpeedInMin;
    }
}

template<typename T>
void RaceCar::getParam(const ros::NodeHandle &handle, const std::string &name, T* storage) const
{
    if (!handle.getParam(name, *storage))
        ROS_ERROR("Failed to get parameter \"%s\" from server\n", name.data());
}

template<typename T> 
void RaceCar::getParam(const ros::NodeHandle &handle, const std::string &name,
                            const T &defaultValue, T* storage) const
{
    if (!handle.param<T>(name, *storage, defaultValue))
        ROS_WARN_STREAM("Failed to get parameter " << name.data() << " from server, setting default: " << defaultValue);
}


void RaceCar::Do(const sgtdv_msgs::Control::ConstPtr &controlMsg) {
    std_msgs::Float64 motorSpeedMsg, servoPositionMsg;
    
    // speed command
    motorSpeedMsg.data = controlMsg->speed * m_params.k + m_params.q;
    if (motorSpeedMsg.data > m_params.cmdSpeedOutMax)
        motorSpeedMsg.data = m_params.cmdSpeedOutMax;
    m_publisherMotorSpeedCmd.publish(motorSpeedMsg);

    // steering command
    servoPositionMsg.data = m_params.cmdSteerOutGain * controlMsg->steeringAngle + m_params.cmdSteerOutOffset;
    if (servoPositionMsg.data > m_params.cmdSteerOutMax)
        servoPositionMsg.data = m_params.cmdSteerOutMax;
    else if (servoPositionMsg.data < m_params.cmdSteerOutMin)
        servoPositionMsg.data = m_params.cmdSteerOutMin;
    m_publisherServoPositionCmd.publish(servoPositionMsg);
}

#ifdef VESC_ODOMETRY
    // vesc_to_odom_node won't start publishing /odom topic before it receives servo command
    void RaceCar::init()
    {
        std_msgs::Float64 servoPosition;
        servoPosition.data = m_params.cmdSteerOutOffset;
        m_publisherServoPositionCmd.publish(servoPosition);
    }
    
    void RaceCar::setPublisherPose(const ros::Publisher &posePub)
    {
        m_publisherPoseEstimate = posePub;
    }

    void RaceCar::setPublisherVelocity(const ros::Publisher &velPub)
    {
        m_publisherVelocityEstimate = velPub;
    }

    void RaceCar::odomCallback(const nav_msgs::Odometry &odomMsg)
    {
        sgtdv_msgs::CarPose carPose;
        carPose.position.header = odomMsg.header;
        carPose.position.x = odomMsg.pose.pose.position.x;
        carPose.position.y = odomMsg.pose.pose.position.y;
        const auto z = odomMsg.pose.pose.orientation.z;
        tf::Quaternion orientationQ;
        tf::quaternionMsgToTF(odomMsg.pose.pose.orientation, orientationQ);
        {
            static double roll = 0.0, pitch = 0.0, yaw = 0.0;
            tf::Matrix3x3(orientationQ).getRPY(roll, pitch, yaw);
            carPose.yaw = yaw;
        }
        carPose.covariance = odomMsg.pose.covariance;

        sgtdv_msgs::CarVel carVel;
        carVel.speed = odomMsg.twist.twist.linear.x;
        carVel.yawRate = odomMsg.twist.twist.angular.z;

        m_publisherPoseEstimate.publish(carPose);
        m_publisherVelocityEstimate.publish(carVel);
    }
#endif // VESC_ODOMETRY