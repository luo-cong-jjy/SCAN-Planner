/// @file
/// FSM 对规划模块的统一接口，数据结构定义在 plan_container.hpp，执行实现见 planner_manager.cpp。
/// bool 成功返回用于决定是否发布新轨迹，调用者负责重试/急停及 ROS 消息封装。
#ifndef _PLANNER_MANAGER_H_
#define _PLANNER_MANAGER_H_

#include <stdlib.h>

#include <bspline_opt/bspline_optimizer.h>
#include <bspline_opt/uniform_bspline.h>
#include <scan_planner/DataDisp.h>
#include <plan_env/grid_map.h>
#include <plan_manage/plan_container.hpp>
#include <ros/ros.h>
#include <traj_utils/planning_visualization.h>

namespace scan_planner
{

  // Fast Planner Manager
  // Key algorithms of mapping and planning are called

  class SCANPlannerManager
  {
    // SECTION stable
  public:
    // 无参构造函数声明，定义见 planner_manager.cpp；只构造管理器对象，算法模块由 initPlanModules() 建立。
    // 当前构造函数不接收 nh；调用方在随后 initPlanModules(nh, vis) 中传入句柄并完成业务配置。
    SCANPlannerManager();
    // 析构函数声明，定义在 cpp；管理器销毁后，其地图、优化器和显示入口等成员释放各自所有权。
    ~SCANPlannerManager();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /* main planning interface */
    /// @brief 局部规划入口：输入世界位置与首尾导数，flag_polyInit/flag_randomPolyTraj 选择初始化策略。
    bool reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc,
                       Eigen::Vector3d end_pt, Eigen::Vector3d end_vel, bool flag_polyInit, bool flag_randomPolyTraj);
    bool EmergencyStop(Eigen::Vector3d stop_pos);
    // 参数按位置对应：1 起点位置、2 起点速度、3 起点加速度、4 终点位置、5 终点速度、6 终点加速度。
    // 即使第 3/5/6 个实参都是零向量，也分别绑定到 start_acc/end_vel/end_acc，不按数值内容区分用途。
    bool planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                        const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);
    bool planGlobalTrajWaypoints(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                 const std::vector<Eigen::Vector3d> &waypoints, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);

    // 由 FSM 在创建管理器后显式调用，加载参数并建立地图、优化器和 A*，不属于构造函数。
    // nh 接入当前规划节点的私有参数与 ROS 接口；vis 接收 FSM 已创建的显示对象，内部保存后供管理器复用。
    void initPlanModules(ros::NodeHandle &nh, PlanningVisualization::Ptr vis = NULL);

    // 规划参数缓存，initPlanModules() 从私有参数读入，供时间分配、初始化和结果检查使用。
    PlanParameters pp_;
    // 当前成功局部轨迹，updateTrajInfo() 更新，供 FSM 发布、执行判断和碰撞采样。
    LocalTrajData local_data_;
    // 当前任务的多项式参考，planGlobalTraj*() 更新，供 FSM 选择局部目标。
    GlobalTrajData global_data_;
    /// @brief 与优化器、A* 共享的局部占用地图；地图由管理器初始化，查询接口本身不经 ROS 服务调用。
    GridMap::Ptr grid_map_;

  private:
    /* main planning algorithms & modules */
    // initPlanModules() 接收 FSM 创建的显示入口，发布初始化、A* 引导和优化结果。
    PlanningVisualization::Ptr visualization_;

    // 局部样条优化入口，初始化时接入同一 GridMap 和 A*，执行 Rebound/Refine。
    // 此优化器由当前 Manager 创建、复用并负责释放，未交给其他模块共同持有，因此采用独占所有权。
    // 优化器内部共享地图不等于优化器自身也要共享；Manager 析构/reset 时自动结束它的生命周期。
    BsplineOptimizer::Ptr bspline_optimizer_rebound_;

    // 管理器局部规划连续失败计数，初始 0；各失败返回处递增，成功保存轨迹后清零，用于调节随机中间点扰动幅度。
    // 区别于 FSM 的 replan_fail_count_：本计数按管理器尝试记录，后者用于流程层失败上限/停车决策。
    int continuous_failures_count_{0};

    void updateTrajInfo(const UniformBspline &position_traj, const ros::Time time_now);
    /// @brief 最终采样检查速度/加速度范数；碰撞检查位于优化器与 FSM 安全回调，不在本函数中。
    bool checkDynamicFeasibility(UniformBspline position_traj);

    void reparamBspline(UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative, double ratio, Eigen::MatrixXd &ctrl_pts, double &dt,
                        double &time_inc);

    bool refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points);

    // !SECTION stable

    // SECTION developing

  public:
    // Ptr 是 std::unique_ptr<SCANPlannerManager> 的公开类型别名，typedef 本身不创建指针或管理器对象。
    // 等价写法：using Ptr = std::unique_ptr<SCANPlannerManager>;；Ptr 是项目命名约定，不是 C++ 关键字。
    //
    // 本项目为何独占：SCANReplanFSM 的 planner_manager_ 负责持有这一份管理器，当前不需要多个模块共同拥有它。
    // 状态机通过管理器组织规划；管理器内部的地图/显示可以另行共享，不代表管理器本身也需要共享所有权。
    // 与裸指针手动 new/delete 相比，独占指针用 RAII 明确释放责任，正常销毁或异常展开时自动清理所持对象。
    //
    // 使用过程：FSM 构造时 planner_manager_ 默认为空；init() 中 reset(new SCANPlannerManager) 才创建并接管对象。
    // 随后的 initPlanModules(nh, visualization_) 再配置算法模块；“指针已接管对象”与“模块已配置”是两步。
    // 访问有效对象用 planner_manager_->成员；空指针不可解引用，借用其 get() 返回值的代码不能自行 delete。
    // reset() 清空当前指针并销毁原来独占的对象；reset(new ...) 先接管新对象并释放旧对象的所有权。
    // 指针析构时由默认删除器 delete 所持对象，触发管理器析构及其成员的析构，各成员再释放自己的资源所有权。
    //
    // 独占语义：不能复制此指针；以 std::move 传入移动构造/赋值后，由接收方持有，源 unique_ptr 为空。
    // 此类型不靠共享引用计数管理寿命；使用独占指针不妨碍其他代码在有效生命周期内借用对象。
    // 这是当前可空、在 init 中创建的所有权设计，并非管理器只能用指针表示；按值成员也是另一种设计选择。
    // 对照共享显示对象及示例：docs/面试/C++相关知识.md#智能指针与所有权。
    typedef unique_ptr<SCANPlannerManager> Ptr;

    // !SECTION
  };
} // namespace scan_planner

#endif
