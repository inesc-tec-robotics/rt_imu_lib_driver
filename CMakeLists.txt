cmake_minimum_required(VERSION 2.8.3)
project(rt_imu_lib_driver)


#######################################################################################################################
## packages
#######################################################################################################################

set(${PROJECT_NAME}_CATKIN_COMPONENTS
    rospy
    sensor_msgs
    geometry_msgs
    tf
)


find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_COMPONENTS})


#######################################################################################################################
## catkin specific configuration
#######################################################################################################################


catkin_package(
    CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_COMPONENTS}
)


include_directories(
    ${catkin_INCLUDE_DIRS}
)
