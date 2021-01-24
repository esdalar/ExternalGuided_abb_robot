
cmake_minimum_required(VERSION 2.8.3)
project(egm_path)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf2_geometry_msgs
  tf2_ros
  tf2
  sensor_msgs
  std_msgs
  fiducial_msgs
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  config/EGMPathParams.cfg
)

catkin_package(INCLUDE_DIRS)

###########
## Build ##
###########

add_definitions(-std=c++11)

include_directories(${catkin_INCLUDE_DIRS})
#include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(egm_path src/egm_path.cpp)

add_dependencies(egm_path ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})

target_link_libraries(egm_path ${catkin_LIBRARIES})

add_dependencies(egm_path ${PROJECT_NAME}_gencfg)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS egm_path
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
) 

###########
## Tests ##
###########

