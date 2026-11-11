/// @file
/// 规划可视化公共接口：目标、参考、初值、控制点和绕行路径分别使用 Marker 发布。
/// 这些接口面向调试观察，不提供导航状态确认或控制指令；参数 id 用于区分同话题上的显示对象。
#ifndef _PLANNING_VISUALIZATION_H_
#define _PLANNING_VISUALIZATION_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <bspline_opt/uniform_bspline.h>
#include <iostream>
#include <traj_utils/polynomial_traj.h>
#include <ros/ros.h>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <stdlib.h>

using std::vector;
namespace scan_planner
{
  class PlanningVisualization
  {
  private:
    // 显示对象自己的 ROS 句柄成员；有参构造中 node=nh 复制句柄，复用同一节点资源，不接管调用方的 nh 对象。
    ros::NodeHandle node;

    ros::Publisher goal_point_pub;
    ros::Publisher global_list_pub;
    ros::Publisher init_list_pub;
    ros::Publisher optimal_list_pub;
    ros::Publisher a_star_list_pub;
    ros::Publisher guide_vector_pub;
    ros::Publisher intermediate_state_pub;

  public:
    // 无参构造函数，已在此处定义；/* args */ 只是注释，等价于空参数列表 ()。
    // 例如 PlanningVisualization a; 调用这一版，函数体为空，成员仍执行各自的默认构造。
    // 它不会自动调用下面的有参构造，因此不会注册目标、参考、初值和优化结果等发布接口。
    PlanningVisualization(/* args */) {}

    // 析构函数，已在此处定义；对象销毁时调用。空函数体之后，成员对象仍会自动析构。
    // 在本项目的共享持有关系中，最后一个共享所有者释放该对象时才发生对象销毁。
    ~PlanningVisualization() {}

    // 有参构造函数的声明，具体实现见 planning_visualization.cpp；负责建立规划显示的 ROS 发布接口。
    // 例如 PlanningVisualization b(nh); 或 new PlanningVisualization(nh) 都选择这一版。
    // 与上面的无参构造同名但参数列表不同，属于构造函数重载，按创建时传入的实参选择。
    // nh 的类型是 ros::NodeHandle：这里只按引用接收已有句柄，用其 advertise 注册显示发布器；创建的仍是本类对象。
    PlanningVisualization(ros::NodeHandle &nh);

    // Ptr 是 std::shared_ptr<PlanningVisualization> 的类型别名，不是显示对象或构造函数；声明 Ptr 默认得到空指针。
    //
    // 本项目为何共享：FSM 显示目标/参考，Manager 显示候选/A* 引导/优化结果，两者当前共同持有同一显示对象。
    // FSM::init() 先用 visualization_.reset(new PlanningVisualization(nh)) 创建对象并注册 ROS 发布接口，
    // 再把共享指针传入 Manager::initPlanModules()；其中 visualization_ = vis 保存另一份共享持有者。
    // 复制的是智能指针的管理状态，两个成员访问同一个对象，不是复制 PlanningVisualization 或再创建一套发布器。
    //
    // 工作原理：非空共享指针关联控制块，记录强所有权等管理信息；复制增加强持有者，析构/reset 减少当前持有关系。
    // 最后一个强所有者释放时才销毁显示对象，其 ROS 发布器等成员随后按自身生命周期规则清理。
    // 因而只清空 FSM 的那一份不会清空 Manager 的那一份，也不一定立即销毁对象；临时参数副本也可能持有它。
    // 不传参数的 reset() 只清空当前指针；reset(p) 接管新对象并放弃当前对旧对象的共享所有权。
    // 若要共享已有对象，应复制已有 shared_ptr，不能对同一个裸指针重复建立独立 shared_ptr 控制块，否则可能重复释放。
    //
    // 当前接口选择“共同拥有”；若只需借用且外部能保证生命周期，也可设计为引用/裸指针，不是多人调用就必须共享指针。
    // 引用计数不使显示对象的数据自动线程安全；强引用环也不会自动回收，需要依据归属关系考虑 weak_ptr 等策略。
    // 与管理器的独占所有权对照：docs/面试/C++相关知识.md#智能指针与所有权。
    typedef std::shared_ptr<PlanningVisualization> Ptr;

    void displayMarkerList(ros::Publisher &pub, const vector<Eigen::Vector3d> &list, double scale,
                           Eigen::Vector4d color, int id);
    void generatePathDisplayArray(visualization_msgs::MarkerArray &array,
                                  const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id);
    void generateArrowDisplayArray(visualization_msgs::MarkerArray &array,
                                   const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id);
    void displayGoalPoint(Eigen::Vector3d goal_point, Eigen::Vector4d color, const double scale, int id);
    void displayGlobalPathList(vector<Eigen::Vector3d> global_pts, const double scale, int id);
    void displayInitPathList(vector<Eigen::Vector3d> init_pts, const double scale, int id);
    void displayOptimalList(Eigen::MatrixXd optimal_pts, int id);
    void displayOptimalTraj(UniformBspline position_traj, int id);
    void displayAStarList(std::vector<std::vector<Eigen::Vector3d>> a_star_paths, int id);
    void displayArrowList(ros::Publisher &pub, const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id);
    // void displayIntermediateState(ros::Publisher& intermediate_pub, scan_planner::BsplineOptimizer::Ptr optimizer, double sleep_time, const int start_iteration);
    // void displayNewArrow(ros::Publisher& guide_vector_pub, scan_planner::BsplineOptimizer::Ptr optimizer);
  };
} // namespace scan_planner
#endif
