/*****************************************************/
//Organization: Stuba Green Team
//Authors: Matej Dudák, Patrik Knaperek
/*****************************************************/

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <sgtdv_msgs/Control.h>
#include <sgtdv_msgs/CarPose.h>
#include <sgtdv_msgs/CarVel.h>

#define VESC_ODOMETRY

class RaceCar {
public:
    RaceCar(ros::NodeHandle &handle);

    void setPublisherMotorSpeedCmd(const ros::Publisher &motorSpeedPub)
    {
        m_publisherMotorSpeedCmd = motorSpeedPub;
    };
    void setPublisherServoPositionCmd(const ros::Publisher &servoPositionPub)
    {
        m_publisherServoPositionCmd = servoPositionPub;
    };
    struct Params
    {
        float cmdSpeedInMax;
        float cmdSpeedInMin;
        float cmdSpeedOutMax;
        float cmdSpeedOutMin;
        float cmdSteerInMax;
        float cmdSteerInMin;
        float cmdSteerOutMax;
        float cmdSteerOutMin;
        float cmdSteerOutOffset;
        float cmdSteerOutGain;
        float k;
        float q;
    };

    void setParams(const Params &params)
    {
        m_params = params;
    };

    void Do(const sgtdv_msgs::Control::ConstPtr &controlMsg);
#ifdef VESC_ODOMETRY
    void init();
    void setPublisherPose(const ros::Publisher &posePub);
    void setPublisherVelocity(const ros::Publisher &velPub);
    void odomCallback(const nav_msgs::Odometry &odomMsg);
#endif

    virtual ~RaceCar();

private:
    void LoadParams(ros::NodeHandle &handle);
    template<typename T> void getParam(const ros::NodeHandle &handle, const std::string &name, T* storage) const;
    template<typename T> void getParam(const ros::NodeHandle &handle, const std::string &name,
                                        const T &defaultValue, T* storage) const;
    ros::Publisher m_publisherMotorSpeedCmd;
    ros::Publisher m_publisherServoPositionCmd;
    
    Params m_params;

#ifdef VESC_ODOMETRY
    ros::Publisher m_publisherPoseEstimate;
    ros::Publisher m_publisherVelocityEstimate;
#endif
};
