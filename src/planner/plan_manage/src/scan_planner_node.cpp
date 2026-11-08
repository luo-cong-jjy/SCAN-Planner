/// @file
/// 规划节点可执行文件入口，对应 advanced_param.xml 中的 type="scan_planner_node"。
/// 算法实现位于 scan_replan_fsm.cpp、planner_manager.cpp 及其链接库，入口只负责 ROS 生命周期。

// ROS C++ 的公共头文件，间接包含 ros::init、ros::NodeHandle、ros::Duration、ros::spin 的声明。
// 编译器通过这些声明检查调用；对应实现由构建时链接的 roscpp 库提供。
#include <ros/ros.h>

// ROS 可视化 Marker 消息的类型声明；沿用原有包含，本入口没有直接创建 Marker 对象。
#include <visualization_msgs/Marker.h>

// 引入本项目的 SCANReplanFSM 完整类定义和成员函数接口，后面才能声明该类型的对象。
// include 在编译前的预处理阶段展开头文件内容，不表示运行或启动这个头文件。
// 查找路径来自 CMake 的 include_directories：
//   plan_manage/include + plan_manage/scan_replan_fsm.h → 实际头文件。
// 成员函数实现主要在 scan_replan_fsm.cpp，作为独立源文件编译后与本入口链接成同一个程序。
#include <plan_manage/scan_replan_fsm.h>

// SCANReplanFSM 定义在 scan_planner 命名空间中。
// 此声明允许下面简写为 SCANReplanFSM；完整类型名是 scan_planner::SCANReplanFSM。
using namespace scan_planner;

/// @brief 建立私有参数句柄并初始化状态机，进入 ROS 回调循环；这里没有独立的规划 while 循环。
// main 是程序入口，程序启动后由运行环境把命令行参数传进来，不需要自己调用 main。
// argc：命令行参数项数，包括 argv[0] 中的程序名称/路径。
// argv：指向命令行字符串指针序列；这里 char **argv 与形参写法 char *argv[] 等价。
//   char      ：一个字符，例如 'h'。
//   char *    ：字符指针，可指向以 '\0' 结尾的字符串。
//   char **   ：指向字符指针的指针；argv[i] 取出第 i 个字符串的起始地址。
// 例如运行 ./demo hello 123，则 argc=3：
//   argv[0] → "./demo"，argv[1] → "hello"，argv[2] → "123"。
// argv[1][0] 是字符 'h'；argv[argc] 为 nullptr，不再是一项有效参数字符串。
// 使用 roslaunch 启动时，还可能收到 __name:=... 等 ROS 专用参数，由 ros::init 处理。
int main(int argc, char **argv)
{
  // 初始化当前规划进程的 ROS 客户端状态，解析启动参数与名称重映射。
  // 默认节点名为 scan_planner_node，advanced_param.xml 的 name 可通过启动重映射确定最终名称。
  ros::init(argc, argv, "scan_planner_node");

  // 建立规划器私有操作入口，传给状态机以读取本节点参数、注册回调和发布结果。
  // 相对键 fsm/navi_mode 在默认命名下解析为 /scan_planner_node/fsm/navi_mode，
  // 对应 advanced_param.xml 的 node 内配置；以 / 开头的绝对接口仍使用原绝对名称。
  ros::NodeHandle nh("~");

  // 创建本项目的状态机对象：前者是类型，scan_replan 是对象名；此时还没有完成 init(nh) 的配置。
  SCANReplanFSM scan_replan;

  // 配置本项目状态机：加载参数，创建地图/优化模块，连接目标、位姿与控制器反馈，并注册定时器。
  // 各成员的项目职责见 scan_replan_fsm.h，实际初始化和运行逻辑见 scan_replan_fsm.cpp。
  scan_replan.init(nh);

  // 创建一个 1.0 s 的 ROS 时长对象并等待；这句是等待，不负责反复执行规划回调。
  ros::Duration(1.0).sleep();

  // 进入 ROS 回调循环，按订阅消息和定时器事件调用已注册的函数，直到节点关闭才返回。
  // 这让 main 中创建的状态机对象在节点运行期间保持存活；普通回调在当前线程依次调度。
  // 耗时规划会影响同一回调队列中其他回调的实际响应时刻。
  ros::spin();

  // main 返回 0 表示程序正常结束；局部对象随后按 C++ 生命周期规则销毁。
  return 0;
}
