# laser_pong


Assumptions:
 - LIDAR must be 360 deg and placed where supposed middle of the field is

Modified laser pong must be installed at a branch feature-laserpong

    cd WS_PATH/src
    git clone https://github.com/JanezCim/visualization_tutorials.git -b feature-laserpong

    rosrun laser_pong laser_pong_node.py 
    rosrun interactive_marker_tutorials pong 

# follow instructions on https://github.com/YDLIDAR/ydlidar_ros_driver to install ydlidar driver
roslaunch ydlidar_ros_driver X4.launch
rviz


# laser_music

sudo apt install python3-pygame