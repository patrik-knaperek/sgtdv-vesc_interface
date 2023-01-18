/*****************************************************/
//Organization: Stuba Green Team
//Authors: Matej Dudák
/*****************************************************/

#include "../include/RaceCar.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "raceCar");
    ros::NodeHandle handle;

    std::string commandTopic;
    handle.getParam("command_topic", commandTopic);

    float steeringOffset = 0;
    handle.getParam("/racecar/steering_angle_offset", steeringOffset);
    float steeringRange = 0;
    handle.getParam("/racecar/steering_range_range", steeringRange);
    float maxSpeed = 0;
    handle.getParam("/racecar/speed_max", maxSpeed);
    RaceCar raceCarObj(steeringOffset, steeringRange, maxSpeed);

    raceCarObj.setPublisherMotorSpeed(handle.advertise<std_msgs::Float64>("commands/motor/speed", 1));
    raceCarObj.setPublisherServoPosition(handle.advertise<std_msgs::Float64>("commands/servo/position", 1));

    ros::Subscriber pathTrackingSub = handle.subscribe(commandTopic, 1, &RaceCar::Do, &raceCarObj);

    ros::spin();

    return 0;
}