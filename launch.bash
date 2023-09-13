sudo chmod 666 /dev/vesc #enable driver port
source devel/setup.bash
roslaunch racecar racecar_control_interface.launch
