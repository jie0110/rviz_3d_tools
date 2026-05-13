cmake_minimum_required(VERSION 3.0.2)
project(rviz_3d_tools)

# ── C++14 ────────────────────────────────────────────────
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── 依赖 ─────────────────────────────────────────────────
find_package(catkin REQUIRED COMPONENTS
  rviz
  pluginlib
  geometry_msgs
  roscpp
)

# Qt5（Noetic 默认）
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(OpenGL REQUIRED)
set(CMAKE_AUTOMOC ON)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS rviz pluginlib geometry_msgs roscpp
)

# ── 头文件路径 ────────────────────────────────────────────
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# ── 编译共享库 ────────────────────────────────────────────
add_library(${PROJECT_NAME}
  src/pose_3d_tool.cpp
  src/goal_3d_tool.cpp
  include/rviz_3d_tools/pose_3d_tool.h
  include/rviz_3d_tools/goal_3d_tool.h
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${rviz_DEFAULT_PLUGIN_LIBRARIES}
  Qt5::Widgets
  OpenGL::GL
)

# ── 安装 ─────────────────────────────────────────────────
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(FILES plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)