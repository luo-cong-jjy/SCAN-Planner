# 启动树对应的 catkin 构建入口：构建规划节点和开/闭环执行、机身仿真、步态显示，并生成 Bspline/DataDisp 消息。
# 修改注释不改变目标；运行时通过 roslaunch 的 pkg/type 定位这里生成的程序。
cmake_minimum_required(VERSION 2.8.3)
# 声明本构建项目名，与 package.xml 的包名及导出目标命名对应。
project(scan_planner)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

# 查找编译/链接所需依赖；找到依赖不代表会启动依赖中的 ROS 节点。
find_package(Eigen3 REQUIRED)
# 查找编译/链接所需依赖；找到依赖不代表会启动依赖中的 ROS 节点。
find_package(PCL 1.7 REQUIRED)

# 查找编译/链接所需依赖；找到依赖不代表会启动依赖中的 ROS 节点。
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf
  plan_env
  path_searching
  bspline_opt
  traj_utils
  message_generation
  cv_bridge
  visualization_msgs
)

# Generate messages in the 'msg' folder
# 登记本包的消息源文件，供后续 generate_messages 生成接口。
add_message_files(
  FILES
  Bspline.msg
  DataDisp.msg
  )
  
# Generate added messages and services with any dependencies listed here
# 按消息依赖生成 C++/Python 等接口；程序需在消息生成目标完成后编译。
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

# 向下游包导出本包头文件、库和 catkin 依赖，供 find_package(catkin COMPONENTS ...) 使用。
# catkin_package(CATKIN_DEPENDS message_runtime)
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES scan_planner
 CATKIN_DEPENDS plan_env path_searching bspline_opt traj_utils message_runtime
#  DEPENDS system_lib
)

# 设置编译头文件搜索路径；SYSTEM 路径按工具链规则处理依赖头的告警。
include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)


# 生成命名的可执行文件，launch 的 type 必须对应此目标生成的程序。
add_executable(scan_planner_node
  src/scan_planner_node.cpp 
  src/scan_replan_fsm.cpp
  src/planner_manager.cpp
  )
# 为当前目标链接所需实现库；头文件可见不等于已经完成符号链接。
target_link_libraries(scan_planner_node 
  ${catkin_LIBRARIES}
  )
# 显式建立构建顺序，保证自定义消息等生成步骤先于使用它们的目标。
add_dependencies(scan_planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS})

# 生成命名的可执行文件，launch 的 type 必须对应此目标生成的程序。
add_executable(open_loop_controller src/open_loop_controller.cpp)
# 为当前目标链接所需实现库；头文件可见不等于已经完成符号链接。
target_link_libraries(open_loop_controller ${catkin_LIBRARIES})
# 显式建立构建顺序，保证自定义消息等生成步骤先于使用它们的目标。
add_dependencies(open_loop_controller ${${PROJECT_NAME}_EXPORTED_TARGETS})

# 生成命名的可执行文件，launch 的 type 必须对应此目标生成的程序。
add_executable(closed_loop_controller src/closed_loop_controller.cpp)
# 为当前目标链接所需实现库；头文件可见不等于已经完成符号链接。
target_link_libraries(closed_loop_controller ${catkin_LIBRARIES})
# 显式建立构建顺序，保证自定义消息等生成步骤先于使用它们的目标。
add_dependencies(closed_loop_controller ${${PROJECT_NAME}_EXPORTED_TARGETS})

# 生成命名的可执行文件，launch 的 type 必须对应此目标生成的程序。
add_executable(go2_kinematic_sim src/go2_kinematic_sim.cpp)
# 为当前目标链接所需实现库；头文件可见不等于已经完成符号链接。
target_link_libraries(go2_kinematic_sim ${catkin_LIBRARIES})

# 生成命名的可执行文件，launch 的 type 必须对应此目标生成的程序。
add_executable(go2_gait_publisher src/go2_gait_publisher.cpp)
# 为当前目标链接所需实现库；头文件可见不等于已经完成符号链接。
target_link_libraries(go2_gait_publisher ${catkin_LIBRARIES})
