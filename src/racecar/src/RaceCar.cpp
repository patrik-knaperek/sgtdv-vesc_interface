/*****************************************************/
//Organization: Stuba Green Team
//Authors: Matej Dudák
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