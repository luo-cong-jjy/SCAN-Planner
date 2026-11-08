# 启动树对应的 catkin 构建入口：构建 PCD 文件场景发布器 map_pub，默认随机地图分支不运行它。
# 修改注释不改变目标；运行时通过 roslaunch 的 pkg/type 定位这里生成的程序。
cmake_minimum_required(VERSION 2.8.3)
# 声明本构建项目名，与 package.xml 的包名及导出目标命名对应。
project(map_generator)

set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_CXX_FLAGS "-std=c++11")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

# 查找编译/链接所需依赖；找到依赖不代表会启动依赖中的 ROS 节点。
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  pcl_conversions
)
# 查找编译/链接所需依赖；找到依赖不代表会启动依赖中的 ROS 节点。
find_package(PCL REQUIRED)
# 查找编译/链接所需依赖；找到依赖不代表会启动依赖中的 ROS 节点。
find_package(Eigen3 REQUIRED)

# 向下游包导出本包头文件、库和 catkin 依赖，供 find_package(catkin COMPONENTS ...) 使用。
catkin_package()

# 设置编译头文件搜索路径；SYSTEM 路径按工具链规则处理依赖头的告警。
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

# 生成命名的可执行文件，launch 的 type 必须对应此目标生成的程序。
add_executable (map_pub src/map_publisher.cpp )
# 为当前目标链接所需实现库；头文件可见不等于已经完成符号链接。
target_link_libraries(map_pub
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES})
