# 启动树对应的 catkin 构建入口：构建备用单目标到路径转换节点；默认 run.launch 不启动该工具。
# 修改注释不改变目标；运行时通过 roslaunch 的 pkg/type 定位这里生成的程序。
cmake_minimum_required(VERSION 2.8.3)
# 声明本构建项目名，与 package.xml 的包名及导出目标命名对应。
project(waypoint_generator)

set(CMAKE_VERBOSE_MAKEFILE "false")
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

set(ADDITIONAL_CXX_FLAG "-Wall -O3 -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${ADDITIONAL_CXX_FLAG}")

# 查找编译/链接所需依赖；找到依赖不代表会启动依赖中的 ROS 节点。
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  nav_msgs
)
# 向下游包导出本包头文件、库和 catkin 依赖，供 find_package(catkin COMPONENTS ...) 使用。
catkin_package()

# 设置编译头文件搜索路径；SYSTEM 路径按工具链规则处理依赖头的告警。
# 设置编译头文件搜索路径；SYSTEM 路径按工具链规则处理依赖头的告警。
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# 生成命名的可执行文件，launch 的 type 必须对应此目标生成的程序。
add_executable(waypoint_generator src/waypoint_generator.cpp)

# 为当前目标链接所需实现库；头文件可见不等于已经完成符号链接。
target_link_libraries(waypoint_generator
   ${catkin_LIBRARIES}
)
