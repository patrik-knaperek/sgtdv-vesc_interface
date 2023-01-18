sudo chmod 666 /dev/ttyACM0 #enable driver port
catkin_make
source devel/setup.sh
roslaunch racecar racecar_control_interface.launch