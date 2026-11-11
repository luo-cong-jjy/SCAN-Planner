/// @file
/// 规划数据容器：GlobalTrajData 保存参考与局部拼接时间关系，LocalTrajData 保存当前发布轨迹。
/// 容器不是 ROS 节点；求值时间的含义由各接口定义，调用者需保证轨迹已初始化且采样区间有效。
#ifndef _PLAN_CONTAINER_H_
#define _PLAN_CONTAINER_H_

#include <Eigen/Eigen>
#include <vector>
#include <ros/ros.h>

#include <bspline_opt/uniform_bspline.h>
#include <traj_utils/polynomial_traj.h>

using std::vector;

namespace scan_planner
{

  /// @brief 全局多项式参考与可选局部样条的统一访问层，保留时间调整量以转换局部/全局参数。
  class GlobalTrajData
  {
  private:
  public:
    // 任务的多项式参考，Manager::planGlobalTraj*() 构造，setGlobalTraj() 保存；坐标 m、时间 s。
    PolynomialTraj global_traj_;
    // 保留的参考拼接段，约定 [0/1/2] 为位置/速度/加速度；setGlobalTraj() 清空。
    // 当前主调用链没有调用 setLocalTraj()，不要与真正执行的 Manager::local_data_ 混为一谈。
    vector<UniformBspline> local_traj_;

    // 参考的有效总时长（s），初值由 global_traj_.getTimeSum() 得到；目标回退时 FSM 可缩短它。
    double global_duration_;
    // setGlobalTraj() 保存的参考创建时刻；局部执行计时使用另一份 LocalTrajData::start_time_。
    ros::Time global_start_time_;
    // 保留拼接段的参考起止时间（s），新参考时置 -1；仅 setLocalTraj() 改为有效区间。
    double local_start_time_, local_end_time_;
    // 拼接造成的累计时间延长（s），新参考置 0；当前位置求值按它补偿全局/局部时间。
    double time_increase_;
    // 最近一次拼接的时间增量（s），新参考置 0；主路径未启用拼接时保持 0。
    double last_time_inc_;
    // 新参考置 0，FSM 选局部目标后写入对应参考时间（s）；当前 getLocalTarget() 每次仍重新投影搜索。
    double last_progress_time_;

    GlobalTrajData(/* args */) {}

    ~GlobalTrajData() {}

    bool localTrajReachTarget() { return fabs(local_end_time_ - global_duration_) < 0.1; }

    /// @brief 保存新参考并重置拼接时间及进度信息；后续求值依赖多项式的 init() 已完成。
    void setGlobalTraj(const PolynomialTraj &traj, const ros::Time &time)
    {
      global_traj_ = traj;
      global_traj_.init();
      global_duration_ = global_traj_.getTimeSum();
      global_start_time_ = time;

      local_traj_.clear();
      local_start_time_ = -1;
      local_end_time_ = -1;
      time_increase_ = 0.0;
      last_time_inc_ = 0.0;
      last_progress_time_ = 0.0;
    }

    /// @brief 保存局部位置/速度/加速度样条及替换参考的时间段，累计时间延长量。
    void setLocalTraj(UniformBspline traj, double local_ts, double local_te, double time_inc)
    {
      local_traj_.resize(3);
      local_traj_[0] = traj;
      local_traj_[1] = local_traj_[0].getDerivative();
      local_traj_[2] = local_traj_[1].getDerivative();

      local_start_time_ = local_ts;
      local_end_time_ = local_te;
      global_duration_ += time_inc;
      time_increase_ += time_inc;
      last_time_inc_ = time_inc;
    }

    /// @brief 按 t 所在段选择全局多项式或局部样条，并应用相应时间偏移。
    Eigen::Vector3d getPosition(double t)
    {
      if (t >= -1e-3 && t <= local_start_time_)
      {
        return global_traj_.evaluate(t - time_increase_ + last_time_inc_);
      }
      else if (t >= local_end_time_ && t <= global_duration_ + 1e-3)
      {
        return global_traj_.evaluate(t - time_increase_);
      }
      else
      {
        double tm, tmp;
        local_traj_[0].getTimeSpan(tm, tmp);
        return local_traj_[0].evaluateDeBoor(tm + t - local_start_time_);
      }
    }

    Eigen::Vector3d getVelocity(double t)
    {
      if (t >= -1e-3 && t <= local_start_time_)
      {
        return global_traj_.evaluateVel(t);
      }
      else if (t >= local_end_time_ && t <= global_duration_ + 1e-3)
      {
        return global_traj_.evaluateVel(t - time_increase_);
      }
      else
      {
        double tm, tmp;
        local_traj_[0].getTimeSpan(tm, tmp);
        return local_traj_[1].evaluateDeBoor(tm + t - local_start_time_);
      }
    }

    Eigen::Vector3d getAcceleration(double t)
    {
      if (t >= -1e-3 && t <= local_start_time_)
      {
        return global_traj_.evaluateAcc(t);
      }
      else if (t >= local_end_time_ && t <= global_duration_ + 1e-3)
      {
        return global_traj_.evaluateAcc(t - time_increase_);
      }
      else
      {
        double tm, tmp;
        local_traj_[0].getTimeSpan(tm, tmp);
        return local_traj_[2].evaluateDeBoor(tm + t - local_start_time_);
      }
    }

    // get Bspline parameterization data of a local trajectory within a sphere
    // start_t: start time of the trajectory
    // dist_pt: distance between the discretized points
    /// @brief 按目标半径截取参考段并重采样，追加边界导数；调用者应保证有效点间距及非退化段数。
    void getTrajByRadius(const double &start_t, const double &des_radius, const double &dist_pt,
                         vector<Eigen::Vector3d> &point_set, vector<Eigen::Vector3d> &start_end_derivative,
                         double &dt, double &seg_duration)
    {
      double seg_length = 0.0; // length of the truncated segment
      double seg_time = 0.0;   // duration of the truncated segment
      double radius = 0.0;     // distance to the first point of the segment

      double delta = 0.2;
      Eigen::Vector3d first_pt = getPosition(start_t); // first point of the segment
      Eigen::Vector3d prev_pt = first_pt;              // previous point
      Eigen::Vector3d cur_pt;                          // current point

      // go forward until the traj exceed radius or global time

      while (radius < des_radius && seg_time < global_duration_ - start_t - 1e-3)
      {
        seg_time += delta;
        seg_time = min(seg_time, global_duration_ - start_t);

        cur_pt = getPosition(start_t + seg_time);
        seg_length += (cur_pt - prev_pt).norm();
        prev_pt = cur_pt;
        radius = (cur_pt - first_pt).norm();
      }

      // get parameterization dt by desired density of points
      int seg_num = floor(seg_length / dist_pt);

      // get outputs

      seg_duration = seg_time; // duration of the truncated segment
      dt = seg_time / seg_num; // time difference between two points

      for (double tp = 0.0; tp <= seg_time + 1e-4; tp += dt)
      {
        cur_pt = getPosition(start_t + tp);
        point_set.push_back(cur_pt);
      }

      start_end_derivative.push_back(getVelocity(start_t));
      start_end_derivative.push_back(getVelocity(start_t + seg_time));
      start_end_derivative.push_back(getAcceleration(start_t));
      start_end_derivative.push_back(getAcceleration(start_t + seg_time));
    }

    // get Bspline parameterization data of a fixed duration local trajectory
    // start_t: start time of the trajectory
    // duration: time length of the segment
    // seg_num: discretized the segment into *seg_num* parts
    /// @brief 按指定时长和段数采样，输出样条参数化所需的位置及首尾导数；seg_num 必须有效。
    void getTrajByDuration(double start_t, double duration, int seg_num,
                           vector<Eigen::Vector3d> &point_set,
                           vector<Eigen::Vector3d> &start_end_derivative, double &dt)
    {
      dt = duration / seg_num;
      Eigen::Vector3d cur_pt;
      for (double tp = 0.0; tp <= duration + 1e-4; tp += dt)
      {
        cur_pt = getPosition(start_t + tp);
        point_set.push_back(cur_pt);
      }

      start_end_derivative.push_back(getVelocity(start_t));
      start_end_derivative.push_back(getVelocity(start_t + duration));
      start_end_derivative.push_back(getAcceleration(start_t));
      start_end_derivative.push_back(getAcceleration(start_t + duration));
    }
  };

  /// @brief 限值与容差的配置集合；max_jerk_ 当前仅被读取保存，不表示已启用 jerk 硬约束。
  struct PlanParameters
  {
    /* planning algorithm parameters */
    // 来自 manager/max_vel、max_acc、max_jerk；默认 0.75 m/s、0.5 m/s²、4 m/s³。
    // 前两项参与参考/初值时间分配和限值检查；max_jerk_ 当前只读入保存，没有硬限幅。
    double max_vel_, max_acc_, max_jerk_; // physical limits
    // 来自 optimization/vel_tolerance、acc_tolerance，默认各 1.0 m/s、1.0 m/s²。
    // checkDynamicFeasibility() 将它们加到对应名义上限后检查采样导数范数。
    double vel_tolerance_, acc_tolerance_;
    // manager/control_points_distance，默认 0.2 m；用于估计样条间隔和采样密度，优化后不保证控制点等距。
    double ctrl_pt_dist;                  // distance between adjacient B-spline control points
    // manager/feasibility_tolerance，默认 0.5（比例）；样条导数控制点各轴按名义限值×1.5 检查。
    // 不通过时尝试时间重分配；与上面的加性采样容差分别使用。
    double feasibility_tolerance_;        // permitted ratio of vel/acc exceeding limits
    // manager/planning_horizon，默认 3.5 m；沿用旧轨迹初始化时用它限制异常采样点数。
    double planning_horizon_;

    /* processing time */
    // 保留统计字段，初值 0；当前管理器未写入搜索耗时。实际终端耗时使用 reboundReplan() 的局部变量。
    double time_search_ = 0.0;
    // 保留统计字段，初值 0；当前未累计优化耗时。
    double time_optimize_ = 0.0;
    // 保留统计字段，初值 0；当前未累计时间调整耗时。
    double time_adjust_ = 0.0;
  };

  /// @brief 当前轨迹编号、世界起始时刻、时长、起点及三种导数样条；start_time_ 可因原地转向后移。
  struct LocalTrajData
  {
    /* info of generated traj */

    // Manager 初始化为 0，每次 updateTrajInfo() 后递增；随 Bspline 发布，控制器日志据此关联轨迹。
    int traj_id_;
    // 当前局部位置样条的有效时长（s），updateTrajInfo() 从 getTimeSum() 更新；首条成功轨迹前不可作为有效值。
    double duration_;
    // 保留的历史拼接偏移字段；当前源码未写入或读取，不是现用的执行时间补偿量。
    double global_time_offset; // This is because when the local traj finished and is going to switch back to the global traj, the global traj time is no longer matches the world time.
    // 当前局部轨迹保存时的 ROS 时刻；FSM/开环按 now-start_time_ 求执行时间，原地转向时 FSM 可后移它。
    // 闭环控制器接收消息后另用自己的 exec_time 推进，不直接以此值采样。
    // ros::Time 默认构造为零，保存轨迹后才写入实际时刻；“轨迹相对时间从0开始”不等于此时间戳必须为0。
    ros::Time start_time_;
    // 位置样条在相对时间 0 的位置（世界系 m）；FSM 用它与当前轨迹采样位置比较普通重规划阈值。
    Eigen::Vector3d start_pos_;
    // 当前可执行轨迹：位置、一次导数速度、二次导数加速度，分别为 m、m/s、m/s²。
    // updateTrajInfo() 成功保存后，由 FSM 的执行/安全采样及下一次重规划使用。
    UniformBspline position_traj_, velocity_traj_, acceleration_traj_;
  };

} // namespace scan_planner

#endif
