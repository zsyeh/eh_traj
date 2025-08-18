cmake_minimum_required(VERSION 3.0.2)
project(eh_traj)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  rospy
  std_msgs
  std_srvs
)

catkin_package(
 CATKIN_DEPENDS geometry_msgs mavros_msgs rospy std_msgs std_srvs
)

catkin_install_python(PROGRAMS
  scripts/trajectory_executor.py
  scripts/integration_tester.py
  scripts/dual_uav_tcp_planner.py
  scripts/tcp_slave_bridge.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
