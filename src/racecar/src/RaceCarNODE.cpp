/*****************************************************/
//Organization: Stuba Green Team
//Authors: Matej Dudák, Patrik Knaperek
/*****************************************************/

#include "../include/RaceCar.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "raceCar");
    ros::NodeHandle handle;

    std::string commandTopic;
    handle.getParam("vesc/command_topic", commandTopic);

    RaceCar raceCarObj;

    raceCarObj.setPublisherMotorSpeed(handle.advertise<std_msgs::Float64>("vesc/commands/motor/speed", 1));
    raceCarObj.setPublisherServoPosition(handle.advertise<std_msgs::Float64>("vesc/commands/servo/position", 1));

#ifdef VEHICLE_ODOMETRY
    raceCarObj.setPublisherPose(handle.advertise<sgtdv_msgs::CarPose>("pose_estimate",1));
    raceCarObj.setPublisherVelocity(handle.advertise<sgtdv_msgs::CarVel>("velocity_estimate",1));
#endif

    ros::Subscriber pathTrackingSub = handle.subscribe(commandTopic, 1, &RaceCar::Do, &raceCarObj);

#ifdef VEHICLE_ODOMETRY
    ros::Subscriber odomSub = handle.subscribe("odom", 1, &RaceCar::odomCallback, &raceCarObj);
    raceCarObj.init();
    ros::spinOnce();
#endif

    ros::spin();

    return 0;
}