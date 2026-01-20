cmake_minimum_required(VERSION 3.0.2)
project(slam_controller)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

# 添加Service文件
add_service_files(
  FILES
  StartMapping.srv
  StopMapping.srv
  StartNavigation.srv
  StopNavigation.srv
  GetMapTasks.srv
  SetMapTasks.srv
  GetSlamStatus.srv
  ListMaps.srv
  ProcessMap.srv
)

# 生成消息和服务
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

# 安装launch文件
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# 安装脚本文件
install(PROGRAMS
  scripts/slam_manager.py
  scripts/run_slam_manager.sh
  scripts/utils.py
  scripts/srv_handlers.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

