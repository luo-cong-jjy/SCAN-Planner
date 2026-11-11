/// @file
/// 控制点优化器的内部状态与目标函数接口，实现见 bspline_optimizer.cpp。
/// 控制点按列排列；每点可关联多个避障基点/方向，求解变量仅覆盖非固定边界段。
#ifndef _BSPLINE_OPTIMIZER_H_
#define _BSPLINE_OPTIMIZER_H_

#include <Eigen/Eigen>
#include <path_searching/dyn_a_star.h>
#include <bspline_opt/uniform_bspline.h>
#include <plan_env/grid_map.h>
#include <ros/ros.h>
#include "bspline_opt/lbfgs.hpp"

// 当前优化输入：局部占用查询、3×N 控制点、时间间隔，以及由碰撞段构造的避障方向。
// 输出：优化后的控制点；本路径不以完整 ESDF 作为输入。
namespace scan_planner
{

  // 单轮优化的控制点和避障引导存储；由 initControlPoints() 建立，目标函数迭代读取/更新。
  class ControlPoints
  {
  public:
    // 期望避障投影间距，initControlPoints() 首次从 dist0_ 复制，默认 0.2 m。
    double clearance;
    // 控制点数量 N，resize(size_set) 更新；非固定首尾区域的点才作为求解变量。
    int size;
    // 待优化控制点，形状为 3×N，列依次对应世界 XYZ 点（m）；求解器每轮将变量写回这里。
    Eigen::MatrixXd points;
    // 第 i 项保存该控制点关联的各个避障基点（世界系 m），由碰撞段及 A* 引导构造。
    std::vector<std::vector<Eigen::Vector3d>> base_point; // The point at the start of the direction vector (collision point)
    // 与 base_point 一一对应的单位避障方向，用来计算控制点相对基点的投影间距。
    std::vector<std::vector<Eigen::Vector3d>> direction;  // Direction vector, must be normalized.
    // 当前碰撞段构造时的临时标志；每次使用前由相关算法重置，不表示持久的占用状态。
    std::vector<bool> flag_temp;                          // A flag that used in many places. Initialize it every time before using it.
    // std::vector<bool> occupancy;

    void resize(const int size_set)
    {
      size = size_set;

      base_point.clear();
      direction.clear();
      flag_temp.clear();
      // occupancy.clear();

      points.resize(3, size_set);
      base_point.resize(size);
      direction.resize(size);
      flag_temp.resize(size);
      // occupancy.resize(size);
    }
  };

  /// @brief 将地图查询、A* 绕行和 L-BFGS 目标函数组织为 Rebound/Refine 两阶段优化。
  class BsplineOptimizer
  {

  public:
    BsplineOptimizer() {}
    ~BsplineOptimizer() {}

    /* main API */
    void setEnvironment(const GridMap::Ptr &env);
    void setParam(ros::NodeHandle &nh);
    Eigen::MatrixXd BsplineOptimizeTraj(const Eigen::MatrixXd &points, const double &ts,
                                        const int &cost_function, int max_num_id, int max_time_id);

    /* helper function */

    // required inputs
    void setControlPoints(const Eigen::MatrixXd &points);
    void setBsplineInterval(const double &ts);
    void setCostFunction(const int &cost_function);
    void setTerminateCond(const int &max_num_id, const int &max_time_id);

    // optional inputs
    void setGuidePath(const vector<Eigen::Vector3d> &guide_pt);
    void setWaypoints(const vector<Eigen::Vector3d> &waypts,
                      const vector<int> &waypt_idx); // N-2 constraints at most

    void optimize();

    Eigen::MatrixXd getControlPoints();

    // 碰撞段绕行搜索入口，Manager 初始化时建立 100³ 节点池；成功路径用于生成避障方向。
    // AStar::Ptr 当前定义为 shared_ptr，但本调用链仅由优化器成员持有；Manager 经此成员配置它不等于新增共享所有者。
    // A* 内部共享地图，与 A* 对象自身的所有权是两层关系；现有代码未体现必须多方共有这个搜索器的需求。
    AStar::Ptr a_star_;
    // Refine 的位置参考点，Manager::refineTrajAlgo() 对时间调整后的轨迹采样得到，单位 m。
    std::vector<Eigen::Vector3d> ref_pts_;

    std::vector<std::vector<Eigen::Vector3d>> initControlPoints(Eigen::MatrixXd &init_points, bool flag_first_init = true);
    bool BsplineOptimizeTrajRebound(Eigen::MatrixXd &optimal_points, double ts); // must be called after initControlPoints()
    bool BsplineOptimizeTrajRefine(const Eigen::MatrixXd &init_points, const double ts, Eigen::MatrixXd &optimal_points);

    inline int getOrder(void) { return order_; }

  private:
    // Manager 注入的共享局部地图，供碰撞段识别、引导构造和优化结果碰撞采样。
    GridMap::Ptr grid_map_;

    // 本次求解的停止原因，由距离代价/碰撞引导更新逻辑写入，earlyExit() 读取。
    enum FORCE_STOP_OPTIMIZE_TYPE
    {
      // 0：继续当前求解。
      DONT_STOP,
      // 1：检测到新的有效碰撞引导，要求结束当前求解并重启。
      STOP_FOR_REBOUND,
      // 2：碰撞段无法继续处理等错误，要求结束当前求解。
      STOP_FOR_ERROR
    } force_stop_type_;

    // main input
    // Eigen::MatrixXd control_points_;     // B-spline control points, N x dim
    // 本轮样条时间间隔（s），由管理器传入；用于控制点差分导数、样条重建和碰撞采样。
    double bspline_interval_; // B-spline knot span
    // 历史终点缓存，当前 Rebound/Refine 实现未读取；有效终点约束来自控制点与管理器输入。
    Eigen::Vector3d end_pt_;  // end of the trajectory
    // int             dim_;                // dimension of the B-spline
    //
    // 历史几何引导点接口；当前 Rebound/Refine 不读取，实际绕行引导存入 cps_.base_point/direction。
    vector<Eigen::Vector3d> guide_pts_; // geometric guiding path points, N-6
    // 历史指定途经点接口，当前主优化路径未读取。
    vector<Eigen::Vector3d> waypoints_; // waypts constraints
    // 与历史 waypoints_ 配套的控制点索引，当前主优化路径未读取。
    vector<int> waypt_idx_;             // waypts constraints index
                                        //
    // 历史停止条件接口，当前主路径使用 rebound/refine 中设置的 L-BFGS 配置，不读取这两个字段。
    int max_num_id_, max_time_id_;      // stopping criteria
    // 历史代价选择字段，当前两个阶段直接调用各自 combineCost*()，不读取此选择值。
    int cost_function_;                 // used to determine objective function
    // 历史动态障碍时间字段，当前路径未维护或读取；不能据此认为优化器在预测障碍运动。
    double start_time_;                 // 保留字段，当前未用于动态障碍预测

    /* optimization parameters */
    // optimization/order，缺省 3；用于固定首尾控制点及界定求解变量区域，当前轨迹为三次样条。
    int order_;                    // bspline degree
    /// @brief 平滑项权重；lambda2_/new_lambda2_ 为初始/本轮避障权重，lambda3_ 可行性，lambda4_ 贴合参考。
    // optimization/lambda_smooth，默认 1.0；两个阶段都用于控制点平滑代价。
    double lambda1_;               // jerk smoothness weight
    // lambda2_：optimization/lambda_collision 默认 1.0；new_lambda2_ 是本轮实际避障权重。
    // Rebound 开始复制初始值，碰撞检查未通过而重启时可加倍。
    double lambda2_, new_lambda2_; // distance weight
    // optimization/lambda_feasibility，默认 0.1；惩罚差分速度、加速度超限。
    double lambda3_;               // feasibility weight
    // optimization/lambda_fitness，默认 1.0；仅 Refine 用来贴合 ref_pts_，不是 Rebound 的代价项。
    double lambda4_;               // curve fitting
    // 保留字段，当前主路径未使用；cpp 中分段代价的局部变量 a 与此成员不是同一个量。
    int a;
    //
    // optimization/dist0，默认 0.2 m；用于设置 cps_.clearance，衡量沿避障方向的投影间距。
    double dist0_;             // safe distance
    // optimization/max_vel、max_acc，默认 0.75 m/s、0.5 m/s²；作为软可行性惩罚阈值。
    double max_vel_, max_acc_; // dynamic limits

    // 3×(N-2×order_) 个标量变量；按中间控制点 XYZ 展开，首尾各 order_ 点固定。z 梯度另置零。
    int variable_num_;              // optimization variables
    // 每次求解重置为 0，在 costFunction*() 中随代价求值递增；是求值计数，不等于 FSM 重试次数。
    int iter_num_;                  // iteration of the solver
    // 历史最优变量缓存，当前主路径未写入/读取；结果直接保存在 cps_.points。
    Eigen::VectorXd best_variable_; //
    // 当前 Rebound 重启时置为最大 double，后续未用于筛选结果；不要将它视为有效最终最小代价。
    double min_cost_;               //

    /// @brief 待优化控制点及其局部碰撞引导；投影距离应与普通欧氏障碍距离区分。
    // 当前控制点和避障引导的工作区，由初值构造、代价计算及重启逻辑共用；字段语义见上方定义。
    ControlPoints cps_;

    /* cost function */
    /* calculate each part of cost function with control points q as input */

    static double costFunction(const std::vector<double> &x, std::vector<double> &grad, void *func_data);
    void combineCost(const std::vector<double> &x, vector<double> &grad, double &cost);

    // q contains all control points
    void calcSmoothnessCost(const Eigen::MatrixXd &q, double &cost,
                            Eigen::MatrixXd &gradient, bool falg_use_jerk = true);
    void calcFeasibilityCost(const Eigen::MatrixXd &q, double &cost,
                             Eigen::MatrixXd &gradient);
    void calcDistanceCostRebound(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient, int iter_num, double smoothness_cost);
    void calcFitnessCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient);
    bool check_collision_and_rebound(void);
    double estimateSegmentYaw(const Eigen::Vector3d &from, const Eigen::Vector3d &to) const;
    double estimateControlPointYaw(const Eigen::MatrixXd &q, int id) const;

    static int earlyExit(void *func_data, const double *x, const double *g, const double fx, const double xnorm, const double gnorm, const double step, int n, int k, int ls);
    static double costFunctionRebound(void *func_data, const double *x, double *grad, const int n);
    static double costFunctionRefine(void *func_data, const double *x, double *grad, const int n);

    bool rebound_optimize();
    bool refine_optimize();
    void combineCostRebound(const double *x, double *grad, double &f_combine, const int n);
    void combineCostRefine(const double *x, double *grad, double &f_combine, const int n);

    /* for benchmark evaluation only */
  public:
    // 独占指针别名：当前 Manager 持有并复用一份优化器状态，不需要多个模块共同管理这个优化器的寿命。
    // 不允许复制所有权，可移动；所有者析构/reset 时自动销毁优化器，内部共享地图则按其自己的持有关系释放。
    typedef unique_ptr<BsplineOptimizer> Ptr;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace scan_planner
#endif
