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

    RaceCar raceCarObj;

    raceCarObj.setPublisherMotorSpeed(handle.advertise<std_msgs::Float64>("commands/motor/speed", 1));
    raceCarObj.setPublisherServoPosition(handle.advertise<std_msgs::Float64>("commands/servo/position", 1));

    ros::Subscriber pathTrackingSub = handle.subscribe(commandTopic, 1, &RaceCar::Do, &raceCarObj);

    ros::spin();

    return 0;
}