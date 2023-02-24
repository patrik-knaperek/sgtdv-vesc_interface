/*****************************************************/
//Organization: Stuba Green Team
//Authors: Matej Dudák, Patrik Knaperek
/*****************************************************/

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <sgtdv_msgs/Control.h>
#include <sgtdv_msgs/CarPose.h>
#include <sgtdv_msgs/CarVel.h>

#define VESC_ODOMETRY

constexpr float STEERING_ANGLE_OFFSET = 0.5f;
constexpr float STEERING_ANGLE_OUT_RANGE = 0.35f;
constexpr float STEERING_ANGLE_IN_RANGE = 1.f;
constexpr float SPEED_OUT_MAX = 10000.f;
constexpr float SPEED_OUT_MIN = 900.f;
constexpr float SPEED_IN_MAX = 100.f;
constexpr float SPEED_IN_MIN = 1.f;
constexpr float K = (SPEED_OUT_MAX - SPEED_OUT_MIN) / (SPEED_IN_MAX - SPEED_IN_MIN);
constexpr float Q = SPEED_OUT_MIN - K * SPEED_IN_MIN;

class RaceCar {
public:
    RaceCar();

    void setPublisherMotorSpeed(const ros::Publisher &mPublisherMotorSpeed);

    void setPublisherServoPosition(const ros::Publisher &mPublisherServoPosition);

    void Do(const sgtdv_msgs::Control::ConstPtr &controlMsg);
#ifdef VESC_ODOMETRY
    void init();
    void setPublisherPose(const ros::Publisher &posePub);
    void setPublisherVelocity(const ros::Publisher &velPub);
    void odomCallback(const nav_msgs::Odometry &odomMsg);
#endif

    virtual ~RaceCar();

private:
    ros::Publisher m_publisherMotorSpeed;
    ros::Publisher m_publisherServoPosition;

#ifdef VESC_ODOMETRY
    ros::Publisher m_publisherPoseEstimate;
    ros::Publisher m_publisherVelocityEstimate;
#endif
};
