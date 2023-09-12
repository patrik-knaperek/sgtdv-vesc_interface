/*****************************************************/
//Organization: Stuba Green Team
//Authors: Patrik Knaperek
/*****************************************************/

#include "../include/joy_vesc_interface.h"
#include <std_msgs/Empty.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "joy_vesc_interface");
    ros::NodeHandle handle;

    JoyVescInterface joy_vesc_interface_obj(handle);

    ros::spin();

    return 0;
}
