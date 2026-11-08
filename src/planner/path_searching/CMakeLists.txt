# 启动树对应的 catkin 构建入口：构建局部 A* 搜索库，供 B 样条优化器使用。
# 修改注释不改变目标；运行时通过 roslaunch 的 pkg/type 定位这里生成的程序。
cmake_minimum_required(VERSION 2.8.3)
# 声明本构建项目名，与 package.xml 的包名及导出目标命名对应。
project(path_searching)

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
  rospy
  std_msgs
  visualization_msgs
  plan_env
  cv_bridge
)


# 向下游包导出本包头文件、库和 catkin 依赖，供 find_package(catkin COMPONENTS ...) 使用。
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES path_searching
 CATKIN_DEPENDS plan_env
#  DEPENDS system_lib
)

# 设置编译头文件搜索路径；SYSTEM 路径按工具链规则处理依赖头的告警。
include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
)

# 把列出的源文件编译为供其他目标链接的库，不产生独立 ROS 节点。
add_library( path_searching 
    src/dyn_a_star.cpp
    )
# 为当前目标链接所需实现库；头文件可见不等于已经完成符号链接。
target_link_libraries( path_searching
    ${catkin_LIBRARIES} 
    )  
