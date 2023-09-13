/*****************************************************/
//Organization: Stuba Green Team
//Authors: Matej Dudák, Patrik Knaperek
/*****************************************************/

#include "../include/vesc_interface.h"
#include <std_msgs/Empty.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "sgt_vesc_interface");
    ros::NodeHandle handle;

    VescInterface sgt_vesc_interface_obj(handle);

    ros::spin();

    return 0;
}
