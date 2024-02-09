cmake_minimum_required(VERSION 2.8.3)
project(comau_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(orocos_kdl REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  trajectory_msgs
  actionlib_msgs
  actionlib
  sensor_msgs
  message_generation
  dynamic_reconfigure
  kdl_parser
  kdl_conversions
  comau_traj
)
find_package( Eigen3 REQUIRED )


include_directories(
  include ${catkin_INCLUDE_DIRS}/usr/include/eigen3 ${orocos_kdl_INCLUDE_DIRS})
include_directories(
   include
   include/comau_control
  ${catkin_INCLUDE_DIRS}
)

add_service_files(
 FILES
  IKServiceTotal.srv
    IKService.srv
    IKPoseReference.srv
 )

 add_action_files(
  FILES
    ComauTrajectory.action
  )

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  trajectory_msgs
  sensor_msgs
  actionlib_msgs
)

 generate_dynamic_reconfigure_options(
   cfg/comau_pose.cfg
 )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES comau_control
  CATKIN_DEPENDS
  message_runtime roscpp rospy std_msgs trajectory_msgs actionlib_msgs
  DEPENDS orocos_kdl
)





add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/ComauSmartSix.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

add_library(comauik
  src/${PROJECT_NAME}/ComauIK.cpp)
target_link_libraries(comauik ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

add_library(comautraj
  src/${PROJECT_NAME}/ComauTrajectoryServer.cpp)
target_link_libraries(comautraj ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})
add_dependencies(comautraj comau_control_generate_services_cpp comau_control_generate_messages_cpp)

add_executable(comau_ik_service src/comau_ik_service.cpp)
target_link_libraries(comau_ik_service ${catkin_LIBRARIES} comauik ${orocos_kdl_LIBRARIES})
add_dependencies(comau_ik_service comau_control_generate_services_cpp)



add_executable(testk src/testk.cpp)
add_dependencies(testk ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(testk ${catkin_LIBRARIES} comauik ${orocos_kdl_LIBRARIES})



add_executable(comau_trajectory_follower src/comau_trajectory_follower.cpp)
add_dependencies(comau_trajectory_follower ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(comau_trajectory_follower ${catkin_LIBRARIES})


add_executable(comau_trajectory_matlab src/comau_trajectory_matlab.cpp)
add_dependencies(comau_trajectory_matlab ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(comau_trajectory_matlab ${catkin_LIBRARIES} comautraj comauik)

add_executable(comau_ss_to_ik src/comau_ss_to_ik.cpp)
add_dependencies(comau_ss_to_ik ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(comau_ss_to_ik ${catkin_LIBRARIES})

add_executable(comau_file_to_ik src/comau_file_to_ik.cpp)
add_dependencies(comau_file_to_ik ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(comau_file_to_ik ${catkin_LIBRARIES})

add_executable(kinematics_test src/kinematics_test.cpp)
add_dependencies(kinematics_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(kinematics_test ${catkin_LIBRARIES})

# add_executable(trail_rotation src/trail_rotation.cpp)
# add_dependencies(trail_rotation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(trail_rotation ${catkin_LIBRARIES} ss_exponential_filter comauik)

add_executable(force_reader src/force_reader.cpp)
add_dependencies(force_reader ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(force_reader ${catkin_LIBRARIES} )


# add_executable(move_comau_to_target src/move_comau_to_target.cpp)
# add_dependencies(move_comau_to_target ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(move_comau_to_target ${catkin_LIBRARIES} ss_exponential_filter comauik)

add_executable(comau_trajectory_recorder
  src/comau_trajectory_recorder.cpp
)
add_dependencies(comau_trajectory_recorder ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(comau_trajectory_recorder
  ${catkin_LIBRARIES}
)

# add_executable(comau_sensor_test
#   src/comau_sensor_test.cpp
# )
# add_dependencies(comau_sensor_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(comau_sensor_test
#   ${catkin_LIBRARIES}
# )

add_executable(comau_workspace_reconfigure
  src/comau_workspace_reconfigure.cpp
)
add_dependencies(comau_workspace_reconfigure ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(comau_workspace_reconfigure
  ${catkin_LIBRARIES} comauik ${orocos_kdl_LIBRARIES}
)