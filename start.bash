sudo chmod 666 /dev/ttyACM0 #enable driver port
catkin_make
source /media/xavier/42878bfd-b273-4b67-b59c-6af6f4a4834b/SGT_DV/ros_implementation/devel/setup.bash
source devel/setup.sh
roslaunch racecar racecar_control_interface.launch
