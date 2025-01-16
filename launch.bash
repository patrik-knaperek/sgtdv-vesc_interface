#!/bin/bash

sudo chmod 666 /dev/vesc #enable driver port
source ./devel/setup.bash
roslaunch vesc_interface racecar_control_interface.launch
