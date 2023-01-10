/*****************************************************/
//Organization: Stuba Green Team
//Authors: Matej Dudák
/*****************************************************/

#include "../include/RaceCar.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "raceCar");
    ros::NodeHandle handle;

    std::string commandTopic;
    if (!handle.getParam("command_topic", commandTopic)) {
        ROS_FATAL("Command topic parameter required.");
        ros::shutdown();
        return 1;
    }

    double steeringOffset = 0;
    handle.getParam("/racecar/steering_angle_offset", steeringOffset);
    double steeringRange = 0;
    handle.getParam("/racecar/steering_range_range", steeringRange);
    RaceCar raceCarObj(steeringOffset, steeringRange);

    raceCarObj.setPublisherMotorSpeed(handle.advertise<std_msgs::Float64>("commands/motor/speed", 1));
    raceCarObj.setPublisherServoPosition(handle.advertise<std_msgs::Float64>("commands/servo/position", 1));

    ros::Subscriber pathTrackingSub = handle.subscribe(commandTopic, 1, &RaceCar::Do, &raceCarObj);

    ros::spin();

    return 0;
}