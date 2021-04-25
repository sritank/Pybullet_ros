cmake_minimum_required(VERSION 3.0.2)
project(pb_ros)

find_package(catkin REQUIRED COMPONENTS
   std_msgs
   sensor_msgs
)

catkin_python_setup()
#catkin_create_pkg()
