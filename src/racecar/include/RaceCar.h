/*****************************************************/
//Organization: Stuba Green Team
//Authors: Matej Dudák
/*****************************************************/

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <racecar/Control.h>

class RaceCar {
public:
    RaceCar();
    RaceCar(double angleOffset, double angleRange);

    void setPublisherMotorSpeed(const ros::Publisher &mPublisherMotorSpeed);

    void setPublisherServoPosition(const ros::Publisher &mPublisherServoPosition);

    void Do(racecar::Control control);

    virtual ~RaceCar();

private:
    ros::Publisher m_publisherMotorSpeed;
    ros::Publisher m_publisherServoPosition;
    double m_steeringAngleOffset;
    double m_steeringAngleRange;
};
