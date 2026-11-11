/// @file
/// 任务调度层：由 scan_planner_node.cpp 创建，在 init() 中连接目标、位姿、规划管理器和执行反馈。
/// 本文件只说明各成员在本项目中的职责和数据来源；算法步骤见配套 cpp，数据类型见其所属头文件。
/// 下文“默认”指当前 run.launch → advanced_param.xml 的正常配置，不是缺参时的回退值。
/// 位姿按规划世界系解释；参数缓存应在 init() 后、观测状态应在 have_odom_ 后使用。
#ifndef _SCAN_REPLAN_FSM_H_
#define _SCAN_REPLAN_FSM_H_

// 机身/目标的 XYZ、速度、加速度和朝向数学表示。
#include <Eigen/Eigen>

// 配套实现中用于采样时间、索引和几何尺寸的 min/max 等算法。
#include <algorithm>

// 接收 RViz 目标位姿，供 mode 1 建立导航任务。
#include <geometry_msgs/PoseStamped.h>

// 输出状态迁移与规划调试日志。
#include <iostream>

// 机身状态输入，提供当前位置、朝向和速度。
#include <nav_msgs/Odometry.h>

// mode 3 的参考路径输入，以及 mode 1 内部的目标封装。
#include <nav_msgs/Path.h>

// 保留依赖；当前状态机没有直接订阅 IMU 消息。
#include <sensor_msgs/Imu.h>

// 连接参数服务器、订阅/发布、定时器和 ROS 时钟。
#include <ros/ros.h>

// 接收闭环控制器的轨迹进度冻结反馈。
#include <std_msgs/Bool.h>

// 保留依赖；当前状态机没有使用 Empty 事件接口。
#include <std_msgs/Empty.h>

// 保存预设关键点、活动关键点和路径采样点。
#include <vector>

// 发布目标和机身碰撞几何等 RViz 标记。
#include <visualization_msgs/Marker.h>

// 规划管理器依赖的局部 B 样条优化接口；算法职责在该模块定义处说明。
#include <bspline_opt/bspline_optimizer.h>

// 安全检查及目标调整使用的局部占用查询接口。
#include <plan_env/grid_map.h>

// 向执行端发布规划结果的自定义消息，定义见 msg/Bspline.msg。
#include <scan_planner/Bspline.h>

// 规划调试消息，定义见 msg/DataDisp.msg。
#include <scan_planner/DataDisp.h>

// 状态机调用的规划管理器；负责创建算法模块并组织轨迹生成。
#include <plan_manage/planner_manager.h>

// 规划结果显示接口，与管理器共用一个显示对象。
#include <traj_utils/planning_visualization.h>

using std::vector;

namespace scan_planner
{

  // 接收任务并决定何时生成、重规划、执行或停止；不在本类中直接实现 A* 和数值优化。
  // 状态跨回调保存在 exec_state_；事件与条件驱动迁移，详见 docs/面试/算法原理与面试问题.md#状态机基本原理与项目调度。
  class SCANReplanFSM
  {

  private:

    // 执行状态，init() 从 INIT 开始；目标回调、执行定时器和安全定时器共同驱动迁移。
    // FSM_EXEC_STATE 是枚举类型；未显式指定数值时，枚举项从 0 依次递增。INIT 是枚举常量，"INIT" 才是字符串。
    enum FSM_EXEC_STATE
    {
      // 0：等待机身状态和任务触发；只收到位姿、尚未给目标时可一直停留。
      INIT,
      // 1：无当前任务时等待；有效目标就绪后切换到生成轨迹。
      WAIT_TARGET,
      // 2：从当前起始状态为目标生成轨迹，失败时可更换初始化方式。
      GEN_NEW_TRAJ,
      // 3：在已有任务执行期间重新规划；成功回到 EXEC_TRAJ，失败累计计数。
      REPLAN_TRAJ,
      // 4：检查执行时间、距起终点阈值和关键点切换条件；安全回调另行检查碰撞。
      EXEC_TRAJ,
      // 5：发布停止轨迹，减速后按 fail_safe 和 need_hover_stop_ 决定恢复或等待新目标。
      EMERGENCY_STOP
    };

    // 任务入口选择，数值与 advanced_param.xml 的 fsm/navi_mode 对应。
    enum NAVI_MODE
    {
      // 1：RViz 的 /move_base_simple/goal 单目标，默认模式。
      MANUAL_TARGET = 1,
      // 2：tools/keypoint.yaml 中的有序关键点任务。
      PRESET_TARGET = 2,
      // 3：外部 /initial_path 提供的路线参考。
      REFERENCE_PATH = 3,
    };

    // 规划执行入口：init() 创建，用于生成全局参考、局部避障轨迹和停止轨迹，保存共享地图及轨迹结果。
    // FSM 独占此管理器；指针成员默认空，init() 接管新对象后才可调用；其所有权不允许直接复制。
    SCANPlannerManager::Ptr planner_manager_;

    // 规划显示入口：init() 创建并传给管理器，显示目标、全局参考、初始轨迹、A* 引导和最优轨迹。
    // FSM 与管理器共享同一显示对象；清空一份指针不代表对象立即销毁，需最后一个强所有者释放。
    PlanningVisualization::Ptr visualization_;

    // /planning/data_display 的发送缓存；当前执行回调主要更新时间戳，a~e 未填入业务统计。
    scan_planner::DataDisp data_disp_;

    // 导航模式缓存：读取 fsm/navi_mode，默认 1；决定 init() 注册哪种目标入口。
    int navi_mode_;

    // no_replan_thresh_：fsm/thresh_no_replan，默认 0.1 m，靠近终点时不触发普通重规划。
    // replan_thresh_：fsm/thresh_replan，默认 1.0 m，尚未离开轨迹起点该距离时不普通重规划。
    // 两者比较当前时刻的轨迹采样位置，安全回调仍可单独触发恢复。
    double no_replan_thresh_, replan_thresh_;

    // mode 2 的配置关键点，来自 YAML 的 fsm/waypointN_x/y/z，世界系 m；init() 加载。
    // 每个列表元素是一处 Eigen 三维点；列表首次构造为空，init() 先按点数 resize，再逐轴读取参数填入。
    // 外层 std::vector 管理“有几个点”，内层 Eigen::Vector3d 规定“每点有 XYZ 三个 double”；不要把二者当成同一层。
    // 当前逐轴读取失败会保存 -1.0，未单独校验；真实负坐标也合法，不能用等于 -1 判定读取成败。
    std::vector<Eigen::Vector3d> preset_waypoints_;

    // mode 2 配置点数，来自 fsm/waypoint_num；文件中必须为正，否则节点关闭。其他模式不使用。
    int waypoint_num_;

    // fsm/planning_horizon，默认 3.5 m；getLocalTarget() 沿全局参考累计距离选择局部目标。
    double planning_horizon_;

    // fsm/emergency_time_，默认 1.0 s；安全回调重规划失败后，碰撞剩余时间小于它则急停。
    double emergency_time_;

    // mode 1 首条有效机身 Odometry 的 z，单位 m；随后所有 RViz 单目标沿用该高度，不随每次点击改变。
    double rviz_goal_height_;

    // 显示几何的上下厚度，来自 grid_map/obstacles_inflation_z_up/down，默认各 0.1 m。
    // 供 publishSelfInflationMarker() 绘制圆柱；实际占用膨胀由 GridMap 的同名配置独立维护。
    double self_inflation_z_up_, self_inflation_z_down_;
    // 显示圆柱半径/前后中心偏移，来自 grid_map/double_cylinder_radius/offset，默认 0.25/0.18 m。
    // 两圆柱中心相距 0.36 m，随机身 yaw 排列；此成员本身只负责 FSM 的几何显示。
    double self_double_cylinder_radius_, self_double_cylinder_offset_;

    // grid_map/body_height，默认 0.4 m；仅 mode 3 将它加到输入路径的 z，不改变仿真初始高度。
    // 表达上游路线参考点到机身规划参考点的固定 Z 偏移，不是里程计 z 的测量结果或缺测补偿。
    double body_height_;

    // 机身碰撞 Marker 的参考系，读取 grid_map/frame_id，未配置时为 world。
    std::string self_inflation_frame_id_;

    // init() 将四个标志置 false。trigger_：收到导航触发；have_target_：目标和全局参考已就绪，可继续局部规划。
    // have_target_ 不表示新的局部 B 样条已经生成或发布，局部结果保存在管理器的 local_data_。
    // have_odom_：至少接收过机身状态；它不包含数据超时判断。
    // have_new_target_：全局目标/参考刚更新，下一次局部优化需重建多项式初值；局部尝试后由 callReboundReplan() 清除。
    // 它不是重规划许可：任务执行期间常见 have_target_=true、have_new_target_=false，同样可以重规划。
    bool trigger_, have_target_, have_odom_, have_new_target_;

    // 初值 false；mode 1 首次记录 rviz_goal_height_ 后为 true，之前拒绝 RViz 目标。
    bool rviz_height_ready_;

    // 初值 false；由 /planning/go2_execution_frozen 更新，true 时补偿局部轨迹 start_time_ 以暂停进度。
    bool go2_execution_frozen_;

    // enable_fail_safe_：fsm/fail_safe，默认 true；急停后速度低于 0.1 m/s 时允许自动选择后续状态。
    // need_hover_stop_：初值 false；失败次数达到上限后置 true，停车后等待新目标，而非继续原任务。
    bool enable_fail_safe_, need_hover_stop_;

    // 当前执行状态，init() 置 INIT；由 changeFSMExecState() 修改，执行定时器按它选择处理分支。
    FSM_EXEC_STATE exec_state_;

    // 同一状态连续提交的次数；changeFSMExecState() 在换状态时置 1，相同状态递增，用于初值重试选择。
    int continuously_called_times_{0};
    // 常规新规划/重规划失败计数；相应成功分支清零，finishProcess() 用它触发停止等待。
    // GEN_NEW_TRAJ 与 REPLAN_TRAJ 共用，不按函数分别统计；当前还有切换关键点参考准备失败的增量，不覆盖所有安全回调内尝试。
    int replan_fail_count_{0};
    // fsm/max_replan_fail_count，默认 1000 次；不是秒数或 L-BFGS 迭代次数。
    int max_replan_fail_count_{1000};

    // 上一次调用 updateLocalTrajTimeFreeze() 的 ROS 时刻，init() 首次设置；每次调用均刷新，与冻结标志无关。
    // 当前时刻减去它得到“相邻两次调用的间隔 dt”，不是累计冻结时长；正常时通常为小的正数。
    // 冻结时将有效 dt 加到轨迹起始时刻，使 now-start_time_ 保持不变；0.2 s 只过滤单次大间隔。
    ros::Time last_freeze_update_time_;

    // odom_pos_/odom_vel_：Odometry 回调更新的世界位置（m）和线速度（m/s），供规划起点和恢复判断使用。
    // odom_acc_：保留字段，当前回调没有更新、主路径没有使用；不能将它当作有效测量加速度。
    Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_;

    // 来自机身 Odometry 的旋转四元数；getOdomYaw() 用它确定机身朝向及双圆柱显示方向。
    Eigen::Quaterniond odom_orient_;

    // init_pt_：任务触发时记录的 odom 位置，当前只记录，后续旧用途已注释。
    // start_pt_：本轮规划起点，默认取实际 odom 位置；mode 3 重规划可取旧轨迹采样位置。
    // start_vel_/start_acc_：起始导数（m/s、m/s²），来自 odom/旧轨迹导数；无有效旧轨迹时起始加速度置零。
    // start_yaw_：保留字段，当前赋值与使用已停用，不应读取为有效朝向边界。
    Eigen::Vector3d init_pt_, start_pt_, start_vel_, start_acc_, start_yaw_;
    // end_pt_：当前段最终目标（世界系 m），来自点击、关键点或参考路径，必要时沿参考向后调整。
    // end_vel_：当前回调将其置零，但主规划调用使用独立的局部目标速度，本版本未读取此缓存。
    Eigen::Vector3d end_pt_, end_vel_;
    // 本次局部优化的终端位置/速度，getLocalTarget() 更新并交给管理器；单位 m、m/s。
    // 目标靠近最终点时速度置零，否则取参考速度；位置不可用时沿参考寻找附近可用点。
    Eigen::Vector3d local_target_pt_, local_target_vel_;

    // mode 2 当前执行的关键点副本；任务开始从 preset_waypoints_ 复制，序列结束后清空。
    std::vector<Eigen::Vector3d> active_waypoints_;
    // active_waypoints_ 的 0 起始索引；init()/新任务置 0，靠近当前关键点或该段时间结束时推进。
    int current_wp_;

    // init() 为 true；急停进入后只发布一次停止轨迹，随后置 false；新规划成功或失败上限触发时重新置 true。
    bool flag_escape_emergency_;

    // 保留句柄成员，当前 FSM 实现未直接使用；实际参数读取、订阅和定时器注册使用 init(nh) 的传入句柄。
    ros::NodeHandle node_;

    // exec_timer_：0.01 s，执行状态机；safety_timer_：0.05 s，检查当前轨迹碰撞风险。均在 init() 创建。
    // 这两个成员保存 createTimer() 返回的定时任务句柄，不是计时数值或回调执行结果。
    // 保存句柄以维持任务生命周期；最后一个对应 Timer 句柄释放后，该定时任务自动停止。
    ros::Timer exec_timer_, safety_timer_;

    // 下面四个成员保存本节点的“消息订阅关系”，init() 中接收 nh.subscribe() 返回的订阅句柄。
    // 它们不是目标坐标、路径点列表或布尔开关；消息到达后，由对应回调提取数据、更新任务状态。
    // goal_sub_：手动单目标的接收入口，仅 mode 1 建立；订阅 RViz 的 /move_base_simple/goal。
    //   消息类型 geometry_msgs/PoseStamped，收到后调用 rvizGoalCallback()，把点击目标转为规划任务。
    // path_sub_：外部路线参考的接收入口，仅 mode 3 建立；订阅 /initial_path。
    //   消息类型 nav_msgs/Path，收到后调用 pathCallback()，用一组路径点建立全局参考，供局部避障跟随。
    // mode 1/3 在 init() 中只建立各自的这一种目标订阅；mode 2 读取关键点文件，不建立这两路订阅。
    // odom_sub_：机身状态接收入口，订阅 /body_pose_topic 指定的 Odometry，交给 odometryCallback() 更新位姿/速度。
    // go2_execution_frozen_sub_：控制器反馈入口，接收 /planning/go2_execution_frozen 的 Bool，更新轨迹进度冻结标志。
    ros::Subscriber goal_sub_, odom_sub_, path_sub_, go2_execution_frozen_sub_;

    // bspline_pub_：/planning/bspline，发布成功轨迹或停止轨迹；data_disp_pub_：/planning/data_display 调试数据。
    // self_inflation_pub_：私有 self_inflation，显示两圆柱几何并锁存最后消息。
    // replan_pub_/new_pub_：当前没有 advertise()/publish() 调用，属于保留句柄。
    ros::Publisher replan_pub_, new_pub_, bspline_pub_, data_disp_pub_, self_inflation_pub_;

    // 选择局部目标，调用管理器规划；成功后发布样条和最优轨迹显示，返回本次尝试是否成功。
    bool callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj);
    // 以 stop_pos 构造停止轨迹并发布，供急停状态使用；位置单位 m。
    bool callEmergencyStop(Eigen::Vector3d stop_pos);
    // 为当前任务更新起点与参考并尝试局部重规划，失败时更换初始化；返回是否恢复成功。
    bool planFromCurrentTraj();
    // 新规划前组织起始位置/导数；有效旧轨迹提供速度加速度，导数背离目标时清零。
    void setStartStateFromOdomOrCurrentTraj();

    // 写入新状态并维护连续次数；pos_call 是日志来源标签，如 FSM、TRIG、SAFETY。
    void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);

    // 返回连续状态提交次数及当前状态，GEN_NEW_TRAJ 用它决定是否加入随机初值。
    std::pair<int, SCANReplanFSM::FSM_EXEC_STATE> timesOfConsecutiveStateCalls();
    // 打印当前状态；由执行回调按计数间隔输出，不改变状态。
    void printFSMExecState();

    // mode 2 的任务入口：复制已加载关键点，显示各点并尝试规划首段。
    void planGlobalTrajbyGivenWps();

    // 由一组路径点生成全局多项式参考、更新目标和显示；供 mode 3 使用。
    bool planGlobalTrajByWaypoints(const std::vector<Eigen::Vector3d> &waypoints);
    // mode 2 当前关键点的参考准备入口；首次索引为 0，后续由 EXEC_TRAJ 先增加 current_wp_ 再调用。
    // 函数自身不切换索引或直接生成局部执行轨迹；成功返回后由调用方安排 GEN_NEW_TRAJ。
    bool planNextWaypoint();

    // 当前仅 mode 2 返回 true；mode 3 沿整条参考输入处理，不按同一索引逻辑切点。
    bool isWaypointSequenceMode() const;
    // 终点已占用时沿全局参考向后找可用点，更新 end_pt_ 与参考有效时长；找不到返回 false。
    bool adjustGlobalTargetIfOccupied();
    // 更新 local_target_pt_/vel_，把全局任务缩成当前局部优化终端条件。
    void getLocalTarget();
    // 检查失败上限并触发 EMERGENCY_STOP，设置停车后等待新目标的标志。
    void finishProcess();
    // 按实际机身位姿和显示几何配置发布前后圆柱 Marker，供 RViz 观察碰撞模型。
    void publishSelfInflationMarker();
    // 返回当前机身四元数对应的 yaw（rad），用于显示几何的朝向。
    double getOdomYaw() const;
    // 返回 from→to 的水平朝向（rad），用于轨迹采样的带方向占用检查；水平位移退化时回退机身 yaw。
    double estimateYawFromSegment(const Eigen::Vector3d &from, const Eigen::Vector3d &to) const;
    // 处理原地转向冻结反馈，后移局部轨迹起始时刻，使规划端参考进度暂停。
    void updateLocalTrajTimeFreeze();

    // 状态定时入口：处理任务生成、执行、常规重规划、关键点切换及急停恢复。
    // 注册时 &SCANReplanFSM::execFSMCallback 指定本成员函数，this 指定要处理哪个状态机实例。
    // 实际调用时只接收 ROS 生成的 TimerEvent 事件 e；注册参数里的 this 不是 e。
    void execFSMCallback(const ros::TimerEvent &e);
    // 安全定时入口：采样当前轨迹，发现风险先尝试恢复，再按剩余时间选择重规划或急停。
    void checkCollisionCallback(const ros::TimerEvent &e);

    // mode 1 接收点击目标，首次机身高度就绪后内部转给 waypointCallback()。
    void rvizGoalCallback(const geometry_msgs::PoseStampedConstPtr &msg);
    // mode 1 的内部目标处理函数：取首点 XY 和固定 rviz_goal_height_ 生成参考，当前不单独订阅 Path。
    void waypointCallback(const nav_msgs::PathConstPtr &msg);
    // mode 3 接收 /initial_path，筛点并添加 body_height_ 后建立路线参考。
    void pathCallback(const nav_msgs::PathConstPtr &msg);
    // 更新机身位置/速度/姿态和观测就绪标志，首次确定 mode 1 的目标高度，并刷新机身几何显示。
    void odometryCallback(const nav_msgs::OdometryConstPtr &msg);
    // 保存闭环控制器是否正在冻结执行进度，供下一次时钟补偿使用。
    void go2ExecutionFrozenCallback(const std_msgs::BoolConstPtr &msg);

    // 保留声明，当前没有定义或调用；实际安全检查入口是 checkCollisionCallback()。
    bool checkCollision();

  public:

    // main() 创建本对象；构造函数体为空，任务状态和参数由 init() 建立，不代表所有字段已有有效值。
    SCANReplanFSM()
    {
    }

    // 退出时成员按自身生命周期回收，规划管理器和共享显示等资源随其所有者释放。
    ~SCANReplanFSM()
    {
    }

    // main() 调用的唯一公开配置入口；使用传入私有句柄加载参数、创建模块并注册各回调。
    void init(ros::NodeHandle &nh);

    // 为本对象的 Eigen 成员提供动态分配对齐支持，不参与规划状态或参数计算。
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

}

#endif
