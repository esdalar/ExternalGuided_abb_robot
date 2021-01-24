cmake_minimum_required(VERSION 2.8.3)
project(egm_interface)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  abb_libegm
  tf2_geometry_msgs
  tf2_ros
  tf2
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS roscpp abb_libegm
  INCLUDE_DIRS ../../install_isolated/include/abb_libegm
)

###########
## Build ##
###########

include_directories(
 ${catkin_INCLUDE_DIRS}
 INCLUDE_DIRS ../../install_isolated/include/abb_libegm
)

add_executable(egm_interface src/egm_interface.cpp)
set_target_properties(egm_interface PROPERTIES OUTPUT_NAME egm_interface PREFIX "")
add_dependencies(egm_interface ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(egm_interface ${catkin_LIBRARIES})


#############
## Install ##
#############

### Mark executables and/or libraries for installation
#install(TARGETS egm_interface
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
#
#install(DIRECTORY launch/
#        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
#) 