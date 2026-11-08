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
    PlanningVisualization(ros::NodeHandle &nh);

    // 本类的 Ptr 是共享智能指针；与管理器的 unique_ptr 别名不同，复制它会共享同一显示对象。
    // 供 FSM 和管理器共同持有，最后一个所有者释放指针时才销毁 PlanningVisualization。
    // Ptr 命名的是“管理该类对象的指针类型”，不是实际可视化对象，也不是构造函数名称。
    // shared_ptr 的 reset(p) 接管一个新对象指针并放弃原来的共享所有权；旧对象只在无其他所有者时销毁。
    // 不传实参的 reset() 只清空当前共享指针，不会另外创建可视化对象。
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
