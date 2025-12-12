
github link:
https://github.com/rishimehta17/project2_ws
##Make this change:
in file web_direct_controller:
location:
ur5_imitation/src/ur5_draw/ur5_draw/web_direct_controller.py

##change the line 13:
URDF_PATH = "/home/rishimehta/ur5_imitation/src/ur_description/urdf/ur5_robot.urdf"

replace "/home/rishimehta/" with "where/you/save/the/package/"

###also go in the root of the ros package in terminal and run this to solve dependecies error:
rosdep update
rosdep install --from-paths src --ignore-src -r -y

pip3 install ikpy
pip install --upgrade websockets
pip3 install websockets ikpy matplotlib scipy numpy
###Now build the ros2_package:
colcon build --symlink-install

###source the files:
source /opt/ros/humble/setup.bash
source install/setup.bash

###To run the code:

###Open the index.html file in any browser using open with option by right clicking on it.

###open terminals:
##in terminal 1:

ros2 launch ur_simulation_gazebo ur_sim_control.launch.py ur_type:=ur5

##in terminal 2:
ros2 run ur5_draw live_plotter_multiview 

##in terminal 3:
ros2 run ur5_draw web_direct_controller 
