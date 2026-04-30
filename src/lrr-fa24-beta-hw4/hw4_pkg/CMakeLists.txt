cmake_minimum_required(VERSION 3.0.2)
project(hw4_pkg)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  visualization_msgs
)

catkin_python_setup()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  hw4_pkg/rrt_node.py
  hw4_pkg/rrt_visualizer.py
  hw4_pkg/pure_pursuit.py
  hw4_pkg/pure_pursuit_node.py
  test/rrt_tests.py
  test/pure_pursuit_tests.py
  hw4_pkg/planning/problems.py
  hw4_pkg/planning/RRTTree.py
  hw4_pkg/planning/search.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

