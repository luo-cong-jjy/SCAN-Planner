# 启动树对应的 catkin 构建入口：导出模型资源包；默认链路通过 robot_description 读取 URDF，不在本包编译控制算法。
# 修改注释不改变目标；运行时通过 roslaunch 的 pkg/type 定位这里生成的程序。
cmake_minimum_required(VERSION 2.8.3)

# 声明本构建项目名，与 package.xml 的包名及导出目标命名对应。
project(go2_description)

# 查找编译/链接所需依赖；找到依赖不代表会启动依赖中的 ROS 节点。
find_package(catkin REQUIRED)

# 向下游包导出本包头文件、库和 catkin 依赖，供 find_package(catkin COMPONENTS ...) 使用。
catkin_package()

# 查找编译/链接所需依赖；找到依赖不代表会启动依赖中的 ROS 节点。
find_package(roslaunch)

foreach(dir config launch meshes urdf)
	# 定义安装空间中的程序/资源位置；devel 运行和 install 后运行的路径布局应区分。
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
