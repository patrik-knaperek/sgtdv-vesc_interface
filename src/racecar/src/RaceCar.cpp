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

void RaceCar::Do(const racecar::Control control) {
    std_msgs::Float64 motorSpeedMsg;
    motorSpeedMsg.data = control.speed;
    m_publisherMotorSpeed.publish(motorSpeedMsg);
    std_msgs::Float64 servoPositionMsg;
    servoPositionMsg.data = control.steeringAngle;
    m_publisherServoPosition.publish(servoPositionMsg);
}