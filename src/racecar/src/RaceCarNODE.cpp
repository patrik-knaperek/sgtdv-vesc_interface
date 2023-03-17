/*****************************************************/
//Organization: Stuba Green Team
//Authors: Matej Dudák, Patrik Knaperek
/*****************************************************/

#include "../include/RaceCar.h"
#include <std_msgs/Empty.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "raceCar");
    ros::NodeHandle handle;

    RaceCar raceCarObj(handle);

    raceCarObj.setPublisherMotorSpeedCmd(handle.advertise<std_msgs::Float64>("vesc/commands/motor/speed", 1));
    raceCarObj.setPublisherServoPositionCmd(handle.advertise<std_msgs::Float64>("vesc/commands/servo/position", 1));

    ros::Subscriber pathTrackingSub = handle.subscribe("pathtracking_commands", 1, &RaceCar::Do, &raceCarObj);

#ifdef VESC_ODOMETRY
    raceCarObj.setPublisherPose(handle.advertise<sgtdv_msgs::CarPose>("pose_estimate",1));
    raceCarObj.setPublisherVelocity(handle.advertise<sgtdv_msgs::CarVel>("velocity_estimate",1));
    ros::Publisher resetOdometryPub = handle.advertise<std_msgs::Empty>("reset_odometry",1);

    ros::Subscriber odomSub = handle.subscribe("odom", 1, &RaceCar::odomCallback, &raceCarObj);
    raceCarObj.init();
    resetOdometryPub.publish(std_msgs::Empty());
    ros::spinOnce();
#endif

    while (ros::ok())
    {
        ros::spinOnce();
    }

    return 0;
}
