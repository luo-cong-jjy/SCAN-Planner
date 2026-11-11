/// @file
/// B 样条公共接口，供优化器和执行端重建同一种轨迹表示。
/// 控制点按维度×点数存储；本项目位置轨迹使用 3×N，具体参数与时间含义见下方字段。
#ifndef _UNIFORM_BSPLINE_H_
#define _UNIFORM_BSPLINE_H_

// Eigen 是 C++ 线性代数库；这里用 MatrixXd 保存控制点矩阵，用 VectorXd 保存节点时间或求值结果。
// 类型名区分数值类型与固定/动态尺寸，不表示不同 ROS 消息；速查见 docs/面试/C++相关知识.md#eigen常用数据类型。
#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>

using namespace std;

namespace scan_planner
{
  // An implementation of non-uniform B-spline with different dimensions
  // It also represents uniform B-spline which is a special case of non-uniform
  class UniformBspline
  {
  private:
    // 当前轨迹控制点，setUniformBspline() 复制输入；项目位置样条为 3×N，列为世界 XYZ（m）。
    // 3 行是空间维度，N 列是点数，与样条次数 p_ 不同；这些点通常不是曲线必经点。
    // 导数样条中的同一成员则承载速度/加速度控制点，应按对应导数阶数理解单位。
    Eigen::MatrixXd control_points_;

    /// @brief p 是次数，n+1 是控制点数，m+1 是节点向量长度；本代码将次数参数命名为 order。
    // p_：输入次数，位置轨迹通常 3，求一次导数减 1；n_=控制点数-1，m_=n_+p_+1。
    // n_/m_ 是从 0 编号的最后下标，不是数量；如 6 个控制点、三次，则 n_=5、m_=9，节点数为 10。
    int p_, n_, m_;     // p degree, n+1 control points, m = n+p+1
    // 长度 m_+1 的节点时间向量（s）；先均匀生成，接收 Bspline 时由消息 knots 覆盖。
    // 有效参数范围为 [u_[p_], u_[m_-p_]]，不是使用完整向量的首尾值；边界外节点用于定义基函数。
    Eigen::VectorXd u_; // knots vector
    // 构造时的名义时间间隔（s）；setKnot() 只替换 u_，因此不能仅凭 interval_ 判断实际节点分布。
    // 初始均匀情况下总时长=(N-p_)*interval_；它不是 ROS 回调周期。表示层笔记见 docs/面试/算法原理与面试问题.md#b样条表示与成员变量。
    double interval_;   // knot span \delta t

    // 内部工具：返回导数样条所需的控制点矩阵；公开的 getDerivative() 再用它构造新导数样条。
    Eigen::MatrixXd getDerivativeControlPoints();

    // 管理器 setPhysicalLimits() 设置的名义限值与相对容差，默认配置为 0.75 m/s、0.5 m/s²、0.5。
    // 仅调用该设置后才供 checkFeasibility() 使用；控制器重建用于求值的样条不依赖这些字段。
    double limit_vel_, limit_acc_, feasibility_tolerance_; // （算法代码层面的人为约束）physical limits and feasibility tolerance

  public:
    UniformBspline() {}
    UniformBspline(const Eigen::MatrixXd &points, const int &order, const double &interval);
    ~UniformBspline();

    Eigen::MatrixXd get_control_points(void) { return control_points_; }

    // initialize as an uniform B-spline
    void setUniformBspline(const Eigen::MatrixXd &points, const int &order, const double &interval);

    // get / set basic bspline info

    void setKnot(const Eigen::VectorXd &knot);
    Eigen::VectorXd getKnot();
    Eigen::MatrixXd getControlPoint();
    double getInterval();
    bool getTimeSpan(double &um, double &um_p);

    // compute position / derivative

    Eigen::VectorXd evaluateDeBoor(const double &u);                                               // use u \in [up, u_mp]
    /// @brief 从轨迹起点计时的 t 转为节点参数 u=t+u[p]；单位 s，采样应在有效时长内。
    // 计算已有样条在相对时间 t 的向量值；返回位置还是速度/加速度，取决于当前对象表示哪一种样条。
    // 本接口使用从 0 开始的相对时间，内部再调用 de Boor 求值；不构造新轨迹，也不自动将位置样条求导。
    inline Eigen::VectorXd evaluateDeBoorT(const double &t) { return evaluateDeBoor(t + u_(p_)); } // use t \in [0, duration]
    UniformBspline getDerivative();

    // 3D B-spline interpolation of points in point_set, with boundary vel&acc
    // constraints
    // input : (K+2) points with boundary vel/acc; ts
    // output: (K+6) control_pts
    /// @brief 由位置采样和首尾速度/加速度约束求三次样条控制点，与直接插入采样点不同。
    static void parameterizeToBspline(const double &ts, const vector<Eigen::Vector3d> &point_set,
                                      const vector<Eigen::Vector3d> &start_end_derivative,
                                      Eigen::MatrixXd &ctrl_pts);

    /* check feasibility, adjust time */

    /// @brief 配置相对容差的控制点检查；时间伸缩与最终向量范数采样检查另见管理器。
    void setPhysicalLimits(const double &vel, const double &acc, const double &tolerance);
    bool checkFeasibility(double &ratio, bool show = false);
    void lengthenTime(const double &ratio);

    /* for performance evaluation */

    double getTimeSum();
    double getLength(const double &res = 0.01);
    double getJerk();
    void getMeanAndMaxVel(double &mean_v, double &max_v);
    void getMeanAndMaxAcc(double &mean_a, double &max_a);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
} // namespace scan_planner
#endif
