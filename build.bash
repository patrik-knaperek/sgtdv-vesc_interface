#!/bin/bash

./install_dependencies.sh
source ${SGT_ROOT}/ros_implementation/devel/setup.bash
catkin build -DCMAKE_BUILD_TYPE=Release
