/*****************************************************/
//Organization: Stuba Green Team
//Authors: Matej Dudák, Patrik Knaperek
/*****************************************************/

#include "../include/RaceCar.h"

RaceCar::RaceCar() = default;

RaceCar::~RaceCar() = default;


void RaceCar::setPublisherMotorSpeed(const ros::Publisher &mPublisherMotorSpeed) {
    m_publisherMotorSpeed = mPublisherMotorSpeed;
}

void RaceCar::setPublisherServoPosition(const ros::Publisher &mPublisherServoPosition) {
    m_publisherServoPosition = mPublisherServoPosition;
}

void RaceCar::Do(const sgtdv_msgs::Control::ConstPtr &controlMsg) {
    std_msgs::Float64 motorSpeedMsg;
    motorSpeedMsg.data = controlMsg->speed * K + Q;
    if (motorSpeedMsg.data > SPEED_OUT_MAX)
        motorSpeedMsg.data = SPEED_OUT_MAX;
    m_publisherMotorSpeed.publish(motorSpeedMsg);

    std_msgs::Float64 servoPositionMsg;
    //convert angle to radian with offset 0.5rad = 28.6479deg
    servoPositionMsg.data = (controlMsg->steeringAngle / STEERING_ANGLE_IN_RANGE * STEERING_ANGLE_OUT_RANGE + STEERING_ANGLE_OFFSET);
    //restrict angle range
    if (servoPositionMsg.data > STEERING_ANGLE_OUT_RANGE + STEERING_ANGLE_OFFSET)
        servoPositionMsg.data = STEERING_ANGLE_OUT_RANGE + STEERING_ANGLE_OFFSET;
    if (servoPositionMsg.data < -STEERING_ANGLE_OUT_RANGE + STEERING_ANGLE_OFFSET)
        servoPositionMsg.data = -STEERING_ANGLE_OUT_RANGE + STEERING_ANGLE_OFFSET;
    m_publisherServoPosition.publish(servoPositionMsg);
}

#ifdef VESC_ODOMETRY
    void RaceCar::init()
    {
        std_msgs::Float64 servoPosition;
        servoPosition.data = 0.5*STEERING_ANGLE_OUT_RANGE + STEERING_ANGLE_OFFSET;
        m_publisherServoPosition.publish(servoPosition);
        sleep(1);
        servoPosition.data = -0.5*STEERING_ANGLE_OUT_RANGE + STEERING_ANGLE_OFFSET;
        m_publisherServoPosition.publish(servoPosition);
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
        carPose.yaw = odomMsg.pose.pose.orientation.z;
        carPose.covariance = odomMsg.pose.covariance;

        sgtdv_msgs::CarVel carVel;
        carVel.speed = odomMsg.twist.twist.linear.x;
        carVel.yawRate = odomMsg.twist.twist.angular.z;

        m_publisherPoseEstimate.publish(carPose);
        m_publisherVelocityEstimate.publish(carVel);
    }
#endif