# 启动树对应的 catkin 构建入口：构建 Armadillo 位姿转换工具库，为里程计可视化提供数学接口。
# 修改注释不改变目标；运行时通过 roslaunch 的 pkg/type 定位这里生成的程序。
cmake_minimum_required(VERSION 2.8.3)
# 声明本构建项目名，与 package.xml 的包名及导出目标命名对应。
project(pose_utils)

# 查找编译/链接所需依赖；找到依赖不代表会启动依赖中的 ROS 节点。
find_package(catkin REQUIRED COMPONENTS
  #armadillo
  roscpp
)

# 向下游包导出本包头文件、库和 catkin 依赖，供 find_package(catkin COMPONENTS ...) 使用。
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pose_utils
#  CATKIN_DEPENDS geometry_msgs nav_msgs
#  DEPENDS system_lib
)

# 查找编译/链接所需依赖；找到依赖不代表会启动依赖中的 ROS 节点。
find_package(Armadillo REQUIRED)

# 设置编译头文件搜索路径；SYSTEM 路径按工具链规则处理依赖头的告警。
include_directories(
    ${catkin_INCLUDE_DIRS}
    ${ARMADILLO_INCLUDE_DIRS}
    include
    )

# 把列出的源文件编译为供其他目标链接的库，不产生独立 ROS 节点。
add_library(pose_utils 
   ${ARMADILLO_LIBRARIES}
   src/pose_utils.cpp)
