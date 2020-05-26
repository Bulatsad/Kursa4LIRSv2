cmake_minimum_required(VERSION 2.8.3)
project(omniturtle_control)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
)

find_package(Curses REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(omniturtle_keyboard_node src/keyboard_controller.cpp)
target_link_libraries(omniturtle_keyboard_node
  ${catkin_LIBRARIES}
  ${CURSES_LIBRARIES}
)
add_dependencies(omniturtle_keyboard_node ${catkin_EXPORTED_TARGETS})

add_executable(omniturtle_manager src/manager.cpp src/mecanum_solver.cpp)
target_link_libraries(omniturtle_manager
   ${catkin_LIBRARIES}
)



install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
