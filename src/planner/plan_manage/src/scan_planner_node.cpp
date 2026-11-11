/// @file
/// 规划节点可执行文件入口，对应 advanced_param.xml 中的 type="scan_planner_node"。
/// 算法实现位于 scan_replan_fsm.cpp、planner_manager.cpp 及其链接库，入口只负责 ROS 生命周期。
/// 本轮启动树精读与面试笔记：docs/面试/算法原理与面试问题.md#启动命令与节点入口。

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
  // C++ 语法上是在构造 nh 对象；"~" 是 ROS 私有名称空间标记，不是另建一个节点或启动线程。
  // 普通 NodeHandle 在根名称空间下访问同一相对键会得到 /fsm/navi_mode，可能无法读到本节点的私有参数。
  // 对比示例与面试追问：docs/面试/C++相关知识.md#ros1节点句柄与命名空间。
  ros::NodeHandle nh("~");

  // 创建本项目的状态机对象：前者是类型，scan_replan 是对象名；此时还没有完成 init(nh) 的配置。
  // 它保存导航阶段并调度规划/执行监视/停止；具体求解交给管理器，不是这里直接控制机器人关节。
  // 六个状态、迁移图及最小例子：docs/面试/算法原理与面试问题.md#状态机基本原理与项目调度。
  SCANReplanFSM scan_replan;

  // 配置本项目状态机：加载参数，创建地图/优化模块，连接目标、位姿与控制器反馈，并注册定时器。
  // 各成员的项目职责见 scan_replan_fsm.h，实际初始化和运行逻辑见 scan_replan_fsm.cpp。
  scan_replan.init(nh);

  // 启动阶段只等待一次 1.0 s ROS 时间，然后进入 spin；main 没有 while，不是每轮控制都等待 1 s。
  // 这是额外等待，不会扣除 init() 的耗时来补齐周期；若 init 用 0.2 s，之后仍再等约 1 s。
  // 此等待本身不调度普通回调，也不保证传感器/目标已就绪；具体 1 s 的必要性未在此处说明。
  // 状态/安全回调目标周期在 init() 的 createTimer 中分别设置为 0.01/0.05 s；控制器有自己的定时器。
  // Duration、Rate 与 Timer 的区别：docs/面试/C++相关知识.md#启动延时与循环周期。
  ros::Duration(1.0).sleep();

  // 进入 ROS 回调循环，按订阅消息和定时器事件调用已注册的函数，直到节点关闭才返回。
  // spin 是调度入口而不是回调：循环在 ROS 内部，某个回调执行完会返回调度器，不会重跑上面的 init/sleep。
  // 回调函数体只处理一轮；同一个状态机对象保持存活，后续事件再次调用它，成员状态因此能跨回调保留。
  // 对照手写 while：spin 对应外层调度循环；execFSMCallback 等具体回调才对应循环里调用的 oneStep。
  // 默认主要调入 scan_replan_fsm.cpp 的状态/安全/位姿/目标回调，以及 grid_map.cpp 的地图回调；具体先后由就绪事件决定。
  // 导航周期逻辑在 execFSMCallback，机身速度控制周期则在另一节点 closed_loop_controller 的 cmdCallback。
  // 这让 main 中创建的状态机对象在节点运行期间保持存活；普通回调在当前线程依次调度。
  // 耗时规划会影响同一回调队列中其他回调的实际响应时刻。
  // 面试追问：回调保存的裸 this 不拥有状态机；这里靠作用域内对象继续存活保证其有效，并非注册回调延长了生命周期。
  ros::spin();

  // main 返回 0 表示程序正常结束；局部对象随后按 C++ 生命周期规则销毁。
  return 0;
}
