/*****************************************************/
//Organization: Stuba Green Team
//Authors: Matej Dudák
/*****************************************************/

#include "../include/RaceCar.h"

RaceCar::RaceCar() = default;

RaceCar::RaceCar(double angleOffset, double angleRange) {
    m_steeringAngleOffset = angleOffset;
    m_steeringAngleRange = angleRange;
};

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
    //convert angle to radian with offset 0.5rad = 28.6479deg
    servoPositionMsg.data = (control.steeringAngle + m_steeringAngleOffset) * M_PI / 180;
    //restrict angle range
    if (servoPositionMsg.data > m_steeringAngleRange)
        servoPositionMsg.data = m_steeringAngleRange;
    if (servoPositionMsg.data < -m_steeringAngleRange)
        servoPositionMsg.data = -m_steeringAngleRange;
    m_publisherServoPosition.publish(servoPositionMsg);
}