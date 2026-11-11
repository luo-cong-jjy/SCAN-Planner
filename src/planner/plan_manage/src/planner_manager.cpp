// #include <fstream>
/// @file
/// 规划管理器：被 FSM 调用，在同一进程中组合 GridMap、AStar、B 样条和多项式轨迹。
/// 全局参考提供方向引导，局部 reboundReplan() 负责初始化、避障优化、时间调整和结果保存。
/// 本类本身不是额外 ROS 节点；失败返回值由 FSM 决定重试和恢复。
#include <plan_manage/planner_manager.h>
#include <thread>

namespace scan_planner
{
  // 命名空间可以嵌套：外层 scan_planner 组织规划模块，下面的无名 namespace 限定本翻译单元使用的辅助实现。
  // 匿名命名空间中的函数具有内部链接；其他 .cpp 即使位于 scan_planner 中，也不能直接链接到这里的同名函数。
  // 本文件后续的 reboundReplan() 可直接调用 applyLinearZReference()；它是普通辅助函数，不是管理器成员函数。
  namespace
  {
    /// @brief 按采样点的 XY 累积弧长在起终点 z 之间插值，退化水平长度时改用点序号比例。
    /// 当前局部高度因此受参考约束，而不是独立进行任意三维优化。
    // 对应论文 IV-C.1 的式 (7)～(8)：保留候选路线的 XY 形状，按水平路径进度重新分配各采样点的高度。
    // 当前默认闭环平面仿真中，起点与局部目标通常同高，此公式退化为“将所有候选采样点设为同一高度”。
    // 这仍可消除随机候选等初值产生的 Z 起伏，使规划适合水平移动；起终点本来同高的平坦候选则保持原高度。
    // 输入局部路段确有高差时才分配上升/下降参考；本函数本身不识别楼梯、不规划跨楼层路线，也不让平面仿真获得爬楼能力。
    // points 是局部候选轨迹的 XYZ 采样点列表（m），还不是 B 样条控制点；引用传参使本函数直接修改调用方的点集。
    // start_z/target_z 是局部规划起点/局部目标的世界系高度（m），不一定是整条导航任务的最终目标高度。
    void applyLinearZReference(std::vector<Eigen::Vector3d> &points, const double start_z, const double target_z)
    {
      // 没有采样点就无需处理，也避免后续 front()/back() 访问空列表。
      if (points.empty())
        return;

      // 只有一个点时无法形成路径区间，直接采用起点高度；本退化分支不再同时满足另一个 target_z。
      if (points.size() == 1)
      {
        points.front()(2) = start_z;
        return;
      }

      // accumulated_xy_length[i] 对应论文 l_i：沿列表从首点走到第 i 点的 XY 累积折线长度（m），首项为 0。
      // 按候选路线的水平累计路程占比来分配高差：完成 10% 的水平路程，就分配 10% 的起终点高差。
      // 例如点的水平累计路程为 [0,1,10] m，高度从 0 升到 1 m，则设为 [0,0.1,1] m，而不是按点序号设为 [0,0.5,1] m。
      // 这里的路程来自采样点的 XY 几何计算，不是机器人实际已走距离；按路程分配使每米对应的高度变化一致。
      // 论文式 (7)：accumulated_xy_length[i] 对应 l_i，最后一项为水平总长 L。
      std::vector<double> accumulated_xy_length(points.size(), 0.0);
      for (size_t i = 1; i < points.size(); ++i)
      {
        // 相邻两点只取 XY，计算水平段长并累加；不是到首点的直线距离，也不使用原来的 Z 波动。
        // head<2>() 是 Eigen 的成员函数模板：<2> 指取前两个分量；三维点 (x,y,z) 的这一部分就是 (x,y)，不删除原来的 z。
        accumulated_xy_length[i] = accumulated_xy_length[i - 1] +
                                   (points[i].head<2>() - points[i - 1].head<2>()).norm();
      }

      // total_xy_length 对应论文 L：整条采样折线的水平总长度（m），即累计列表最后一项。
      const double total_xy_length = accumulated_xy_length.back();
      for (size_t i = 0; i < points.size(); ++i)
      {
        // ratio 对应论文 alpha_i，是 0~1 的水平路径进度，无单位；例如累计 1 m / 总长 4 m = 0.25。
        // 总长接近零时，所有点的 XY 几乎重合，改用 i/(点数-1) 分配高度，避免除以零；不代表已规划出可行的竖直运动。
        // 论文式 (8)：ratio 对应 alpha_i，再以 z_s+alpha_i*(z_g-z_s) 覆盖高度；代码对近零总长使用 1e-6 m 阈值。
        const double ratio = total_xy_length > 1e-6
                                 ? accumulated_xy_length[i] / total_xy_length
                                 : static_cast<double>(i) / static_cast<double>(points.size() - 1);
        // 从起点高度分配总高差：完成 25% 的水平路径，就分配 25% 的高差；只覆盖 Z，原 X/Y 坐标保持不变。
        // 若起终点同高，所有采样点均设为该高度；若目标更低，负高差自然生成下降参考。
        points[i](2) = start_z + ratio * (target_z - start_z);
      }

      // 显式固定首尾高度，确保采样点端值精确采用输入的起点/目标高度。
      // vector::front()/back() 分别返回首点/末点的引用，后面的 Eigen 下标 (2) 访问该点的 Z；赋值直接修改原点，不删除元素。
      points.front()(2) = start_z;
      points.back()(2) = target_z;
    }
  } // namespace

  // SECTION interfaces for setup and query

  // 无参构造的类外定义，函数体为空；FSM 随后显式调用 initPlanModules() 创建实际规划模块。
  SCANPlannerManager::SCANPlannerManager() {}

  // 析构时打印退出日志，函数体结束后各成员按 C++ 生命周期规则继续销毁并释放其资源所有权。
  SCANPlannerManager::~SCANPlannerManager() { std::cout << "des manager" << std::endl; }

  /// @brief 读取参数并创建地图、优化器和固定大小 A* 节点池；各模块共用同一地图实例。
  /// max_jerk_ 当前只被保存；可行性相对容差与最终采样的加性容差分别使用。
  void SCANPlannerManager::initPlanModules(ros::NodeHandle &nh, PlanningVisualization::Ptr vis)
  {
    /* read algorithm parameters */

    // nh.param(参数键, 保存变量, 回退值)：先读取参数服务器，键不存在或读取失败时才将最后一项写入变量。
    // pp_ 是规划配置集合；下列“启动默认”来自 advanced_param.xml（当前私有命名空间通常为 /scan_planner_node）。
    // 回退值由项目作者逐项指定，不是 ROS 自动选取，也不覆盖成功读取的配置；具体数值的调参依据未在此给出。
    // -1 用在本应为正的限值/距离上，是不可用的占位值，不表示“不限速/关闭功能”；本函数未统一校验并拒绝这些负值。
    // 1.0、0.0、5.0 则分别提供容差/距离的具体回退；有数值可用不代表已按真实机器人能力标定。

    // 物理含义：名义线速度上限（m/s），描述位置随时间变化的快慢；0.75 m/s 表示匀速 1 s 移动 0.75 m。
    // 用途：估计参考/候选轨迹时长、B 样条节点时间间隔，并设置后续速度检查的名义限值。
    // 启动默认 manager/max_vel=0.75；缺参/读取失败回退 -1.0，负速度限值不可作为有效配置继续计算。
    // 最终速度采样检查另加 vel_tolerance_，所以此值不等于该检查的实际接受门槛，也不会单独直接给机器人发限速指令。
    nh.param("manager/max_vel", pp_.max_vel_, -1.0);
    // 物理含义：名义线加速度上限（m/s²），描述速度变化的快慢；沿同一方向加速 0.5 m/s² 持续 1 s，速度增加 0.5 m/s。
    // 用途：估计加减速所需时间、设置样条检查限值；FSM 还用它估计末段减速距离。
    // 启动默认 manager/max_acc=0.5；缺参/读取失败回退 -1.0。最终加速度采样检查另加 acc_tolerance_。
    nh.param("manager/max_acc", pp_.max_acc_, -1.0);
    // 物理含义：名义 jerk（加加速度）上限（m/s³），即加速度随时间变化的快慢，关联运动变化是否突兀。
    // 启动默认 manager/max_jerk=4；缺参/读取失败回退 -1.0。
    // 当前 max_jerk_ 只读入保存，未实施该数值的限幅/可行性检查；优化器仍有 jerk 平滑代价，二者不是同一件事。
    nh.param("manager/max_jerk", pp_.max_jerk_, -1.0);
    // 物理含义：最终速度范数采样检查允许的额外超限量（m/s），是检查的放宽量，不是速度测量误差或额外目标速度。
    // checkDynamicFeasibility() 的此项门槛为 max_vel_ + vel_tolerance_；默认 0.75+1.0=1.75 m/s，其他检查仍独立生效。
    // 启动配置 optimization/vel_tolerance=1.0；缺参/读取失败也回退 1.0。这是加性容差，不是“放宽 100%”。
    nh.param("optimization/vel_tolerance", pp_.vel_tolerance_, 1.0);
    // 物理含义：最终加速度范数采样检查允许的额外超限量（m/s²）；此项门槛为 max_acc_ + acc_tolerance_。
    // 默认门槛为 0.5+1.0=1.5 m/s²；不是加速度指令，不是比例，也不代表所有其他检查都允许达到这个值。
    // 启动配置 optimization/acc_tolerance=1.0；缺参/读取失败也回退 1.0。
    nh.param("optimization/acc_tolerance", pp_.acc_tolerance_, 1.0);
    // 含义：判断是否需要拉长轨迹时间时，样条导数控制点各轴检查使用的相对容差，无单位。
    // checkFeasibility() 按名义速度/加速度限值*(1+本值)检查，另有 1e-4 数值裕量；它与上述最终范数采样检查分开使用。
    // 启动默认 manager/feasibility_tolerance=0.5，即放宽 50%；缺参/读取失败回退 0.0，即不增加比例放宽。
    // 例如默认速度检查门槛约 0.75*1.5=1.125 m/s（逐轴），不是 0.75+0.5；不通过时尝试时间重分配。
    nh.param("manager/feasibility_tolerance", pp_.feasibility_tolerance_, 0.0);
    // 几何含义：候选采样/B 样条控制点布局使用的目标空间尺度（m），用于决定轨迹表示的疏密。
    // 用途：以“距离尺度/名义速度”估计节点时间间隔，并控制候选重采样密度；数值更小通常带来更多采样/控制变量。
    // 控制点不一定是曲线必经点，优化后也不保证相邻控制点严格等距；此值不是障碍安全距离。
    // 启动默认 manager/control_points_distance=0.2；缺参/读取失败回退 -1.0，负间距不是有效几何尺度。
    nh.param("manager/control_points_distance", pp_.ctrl_pt_dist, -1.0);
    // 几何含义：局部规划范围的距离尺度（m）。此 manager 缓存目前用于“沿用旧轨迹初值”的异常点数判据：
    // point_set.size() > planning_horizon_/ctrl_pt_dist*3 时，认为初值过长并改为多项式初始化。
    // 实际前视局部目标由 FSM 的 fsm/planning_horizon 选择；两个参数键分开读取，默认 launch 用同一 arg 赋值。
    // 启动默认 manager/planning_horizon=3.5；缺参/读取失败回退 5.0，是具体距离尺度，不是无效占位。
    nh.param("manager/planning_horizon", pp_.planning_horizon_, 5.0);

    local_data_.traj_id_ = 0; // 局部轨迹编号从 0 开始，每次保存新轨迹时递增。
    // “局部”指固定尺寸、随机器人平移的三维滑动地图窗口：保留附近体素，滑动时复用移出区域的缓冲区。
    // new GridMap 创建对象并返回地址；共享智能指针的 reset(地址) 接管该对象，释放自己对旧对象的持有关系。
    // 首次初始化通常没有旧对象；若有，须等旧对象最后一个共享持有者释放它时才析构，不会直接清空其他持有者。
    // 此步只创建并接管对象；窗口尺寸/分辨率由下一行初始化，后续地图更新依据位姿决定窗口滑动。
    grid_map_.reset(new GridMap); // 创建局部占用地图对象，由智能指针管理。
    grid_map_->initMap(nh); // 读取地图参数、分配缓冲区，并建立点云/位姿等 ROS 接口。

    // 创建 B 样条优化器，读取代价权重与运动限值，并接入上面创建的同一份地图用于碰撞查询。
    // 这里是独占智能指针的 reset()：接管新优化器；若原来持有旧优化器，会将旧对象销毁。
    bspline_optimizer_rebound_.reset(new BsplineOptimizer);
    bspline_optimizer_rebound_->setParam(nh);
    bspline_optimizer_rebound_->setEnvironment(grid_map_);
    // 创建优化器使用的 A* 搜索器，为碰撞区段寻找绕障引导路径。
    // 此处配置优化器公开的共享指针成员，不在 Manager 中另存一份 AStar 所有权；共享类型不代表实际已多方持有。
    bspline_optimizer_rebound_->a_star_.reset(new AStar);
    // 共用同一份地图，分配 100×100×100 个搜索节点的复用池；这是节点数量，不是 100 m 的地图尺寸。
    // 这里只确定容量；每次 AstarSearch 再设置步长和起终点中点。起终点不是此搜索盒子的最小/最大角。
    // 默认步长取地图分辨率 0.05 m，池每轴名义尺度约 5 m；三维池不代表当前算法自由枚举三维邻居。
    bspline_optimizer_rebound_->a_star_->initGridMap(grid_map_, Eigen::Vector3i(100, 100, 100));

    visualization_ = vis; // 保存 FSM 传入的可视化对象，供管理器显示候选、A* 引导和优化后的轨迹。
  }

  // !SECTION

  // SECTION rebond replanning

  /// @brief 局部规划入口，输入起始位置/速度/加速度及局部目标位置/速度，返回是否得到可发布轨迹。
  /// 先生成初值，再构造碰撞引导并优化，必要时调整时间，最后做动态可行性采样检查。
  // flag_polyInit：是否要求重新构造多项式候选初值；false 时通常利用旧轨迹构造初值，仍会进行局部优化。
  // 这是调用方传入的策略值：FSM 将 have_new_target_ || flag_use_poly_init 作为本参数，不是检测到障碍后自动置 true。
  // GEN_NEW_TRAJ、mode 1/2 的执行中更新均主动要求多项式初值；mode 3 通常先尝试旧轨迹，失败后再要求重建。
  // 首次调用或旧轨迹初值异常时，内部标志也能强制走多项式分支，因此首参 false 不保证一定沿用旧轨迹。
  // flag_randomPolyTraj：仅在多项式初值分支生效；false 用常规单段候选，true 加入带随机偏移的中间点构造候选。
  // 两者选择的是局部规划的初始化策略，不改变起点/局部目标，也不表示是否规划成功；成功与否由函数返回值给出。
  bool SCANPlannerManager::reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
                                        Eigen::Vector3d start_acc, Eigen::Vector3d local_target_pt,
                                        Eigen::Vector3d local_target_vel, bool flag_polyInit, bool flag_randomPolyTraj)
  {

    // count 记录本局部规划入口的调用序号，跨调用保留；日志中的 rebo 可结合函数名理解为 rebound 的缩写标签。
    static int count = 0;
    std::cout << endl
              << "[rebo replan]: -------------------------------------" << count++ << std::endl;
    // 设置 cout 的浮点显示精度：默认浮点格式下为 3 位有效数字（如 1.23456 显示 1.23），并非固定 3 位小数。
    // 只影响打印、不改变计算数据；该设置对后续 cout 持续生效，fixed/scientific 格式下的小数位含义不同。
    cout.precision(3);
    // Eigen::transpose() 返回转置表达式：将 XYZ 列向量按行打印，便于同一行查看；不原地修改位置/速度。
    cout << "start: " << start_pt.transpose() << ", " << start_vel.transpose() << "\ngoal:" << local_target_pt.transpose() << ", " << local_target_vel.transpose()
         << endl;

    if ((start_pt - local_target_pt).norm() < 0.2)
    {
      cout << "Close to goal" << endl;
      // 管理器内部连续失败计数加一；用于后续随机初值扰动幅度，本次完整规划成功时清零。
      // 这是一次失败返回（起终点过近），其他优化/精修等失败返回处也会增加该计数。
      continuous_failures_count_++;
      return false;
    }

    // t_start 是阶段计时起点；后面用 ROS 当前时刻减去它得到耗时，并在进入下一阶段时重新设置。
    ros::Time t_start = ros::Time::now();
    // 三个局部 Duration 变量分别统计本次初始化、优化、精修/检查阶段的计算耗时，toSec() 后单位为 s。
    // 它们用于性能日志，不是轨迹起始时刻、机器人执行时长，也不是从参数服务器读入的配置。
    ros::Duration t_init, t_opt, t_refine;

    /*** STEP 1: INIT ***/
    // 本阶段的输出：point_set（候选曲线的 XYZ 采样点）和 start_end_derivatives（首尾速度/加速度）。
    // 后续将这两组数据与节点时间间隔 ts 一起送入 parameterizeToBspline()，求初始控制点，再进行避障优化。
    // ts（s）是初始采样/样条节点时间间隔，不是 ROS 定时器周期；基本尺度“控制点目标距离 / 名义速度”由 距离=速度×时间 得来。
    // 乘 1.2 或 5 是给初值放宽时间的经验系数，较大的时间间隔通常降低导数幅值，但不保证最终满足速度/加速度限值。
    // 默认初始估计为 0.2/0.75*1.2=0.32 s；后面的重采样可继续缩小 ts。
    // 注意：前面距离<0.2 m 已返回，因此当前正常有效输入会走 >0.1 的分支，*5 分支在此流程下不会用到。
    double ts = (start_pt - local_target_pt).norm() > 0.1 ? pp_.ctrl_pt_dist / pp_.max_vel_ * 1.2 : pp_.ctrl_pt_dist / pp_.max_vel_ * 5;
    // point_set 存位置点（m）；start_end_derivatives 固定按 [起点速度,终点速度,起点加速度,终点加速度] 保存四个三维向量。
    vector<Eigen::Vector3d> point_set, start_end_derivatives;
    // flag_first_call：函数内 static 标志，初始 true，首次进入多项式分支即清为 false；让首次初始化采用多项式策略。
    // 它跨调用保留，未等待规划成功才清除，也不会每次接收目标都重置；新目标通过形参 flag_polyInit 请求重建。
    // flag_force_polynomial：另一 static 标志，初始 false；旧轨迹候选样本过多时置 true，要求下一轮放弃旧候选、重新生成多项式候选。
    // 它只控制上述切换，不保存多种策略编号；多项式候选是否加入随机中间点，仍由 flag_randomPolyTraj 决定。
    static bool flag_first_call = true, flag_force_polynomial = false;
    // 本次调用的外层重试标志：仅在旧轨迹初值异常过长时，让下面的 do-while 清空数据并改走多项式分支。
    bool flag_regenerate = false;
    do
    {
      // 每轮重新构造初值，避免将上一轮候选点/边界条件混入本轮；本循环尚未进入轨迹优化。
      point_set.clear();
      start_end_derivatives.clear();
      flag_regenerate = false;

      // 三个条件任一成立即重建多项式初值；首次调用/强制重建只影响分支选择，不会把形参 flag_polyInit 本身改为 true。
      // 分别对应：首次需要初始化、调用方主动要求重建、旧轨迹候选异常后内部强制重建；三者不是三个顺序执行的阶段。
      // 仅当它们全为 false 时才进入 else，尝试用旧轨迹构造初值；无论选哪个分支，之后都还要进行避障优化。
      if (flag_first_call || flag_polyInit || flag_force_polynomial /*|| ( start_pt - local_target_pt ).norm() < 1.0*/)
      {
        // 已选择多项式策略，消费首次/强制请求；即使后续优化失败，此处也已完成这两个标志的清除。
        flag_first_call = false;
        flag_force_polynomial = false;

        // 这里的“多项式初值”指用多项式生成局部优化的候选曲线；系数由起终点状态及段时长求出，并非随意给系数初值。
        // 此局部变量 gl_traj 连接本次起点与局部目标，后续采样成 point_set，再转换为初始 B 样条控制点 ctrl_pts。
        PolynomialTraj gl_traj;

        // dist 是起点到局部目标的三维直线距离（m），用于估计时间，不是候选曲线的累计长度。
        double dist = (start_pt - local_target_pt).norm();
        // time（s）为整条候选多项式的时长估计，区别于采样间隔 ts 和计算耗时 t_init。
        // 令 v=max_vel、a=max_acc：v²/a 是“从静止加速到 v，再减速到静止”所需距离的尺度。
        // 长距离分支 (dist-v²/a)/v + 2v/a 对应加速、匀速、减速的时间估计。
        // 短距离分支源码用 sqrt(dist/a)，而静止起停的理想对称三角速度总时长为 2*sqrt(dist/a)；此处按原实现作为初值估计。
        // 该估计也未显式代入实际首尾速度，不能视作严格最短时间或运动可行性的证明，后续仍需检查/调整。
        double time = pow(pp_.max_vel_, 2) / pp_.max_acc_ > dist ? sqrt(dist / pp_.max_acc_) : (dist - pow(pp_.max_vel_, 2) / pp_.max_acc_) / pp_.max_vel_ + 2 * pp_.max_vel_ / pp_.max_acc_;

        // flag_randomPolyTraj 是调用方传入的随机候选开关：false 走下面常规单段，true 走 else 的随机中间点方案。
        // 两种方式均只准备局部优化初值，起点和局部目标不变；随机候选仍须优化和检查，不会在此直接执行。
        if (!flag_randomPolyTraj)
        {
          // 非随机候选：用起终点的位置、速度、加速度和给定 time 求单段五次多项式；局部目标加速度取零。
          // one_segment_traj_gen() 返回包含 XYZ 多项式系数和时长的轨迹对象，保存到 gl_traj，供后续按时间采样；函数本身不查询障碍地图。
          gl_traj = PolynomialTraj::one_segment_traj_gen(start_pt, start_vel, start_acc, local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), time);
        }
        else
        {
          // 叉积 cross() 构造与起终点连线垂直的方向，normalized() 将其长度归一为 1，供偏移方向使用。
          // horizon_dir 是水平侧向；vertical_dir 同时垂直于连线和该侧向，连线水平时才是纯竖直方向。
          Eigen::Vector3d horizon_dir = ((start_pt - local_target_pt).cross(Eigen::Vector3d(0, 0, 1))).normalized();
          Eigen::Vector3d vertical_dir = ((start_pt - local_target_pt).cross(horizon_dir)).normalized();
          // 随机中间点 = 起终点中点 + 侧向偏移 + 竖向平面内偏移；两个 rand() 分别产生一次伪随机取样。
          // rand()/RAND_MAX-0.5 给出约 [-0.5,0.5] 的无量纲系数，乘连线长度后成为以 m 为单位的偏移。
          // 0.8/0.4 为两方向的经验幅度系数；f(k)=-0.978/(k+0.989)+0.989 随连续失败次数 k 增大而增大。
          // 因而失败后允许尝试偏得更开的候选，而非始终重复相同曲线；这些系数不代表障碍安全距离。
          Eigen::Vector3d random_inserted_pt = (start_pt + local_target_pt) / 2 +
                                               (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * horizon_dir * 0.8 * (-0.978 / (continuous_failures_count_ + 0.989) + 0.989) +
                                               (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * vertical_dir * 0.4 * (-0.978 / (continuous_failures_count_ + 0.989) + 0.989);
          // pos 的三列依次是起点、随机中间点、局部目标（每列 XYZ）；这里只构造经过三点的候选，不修改实际导航目标。
          Eigen::MatrixXd pos(3, 3);
          pos.col(0) = start_pt;
          pos.col(1) = random_inserted_pt;
          pos.col(2) = local_target_pt;
          // 两个路段各分配 time/2 秒，总时长仍为 time；这不是根据两段实际长度重新分配时间。
          Eigen::VectorXd t(2);
          t(0) = t(1) = time / 2;
          // 根据三点、两段时长及首尾速度/加速度，构造平滑连接的分段五次多项式，作为另一种初始形状。
          gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, local_target_vel, start_acc, Eigen::Vector3d::Zero(), t);
        }

        // 以下按相对时间取样多项式；t 为采样时刻（s），flag_too_far 表示当前时间步长让相邻采样点过远。
        double t;
        bool flag_too_far;
        // 先乘 1.5、进入循环再除 1.5，使第一轮使用原 ts；之后每次失败都将 ts 缩为上一轮的 2/3。
        ts *= 1.5; // ts will be divided by 1.5 in the next
        do
        {
          ts /= 1.5;
          point_set.clear();
          flag_too_far = false;
          // last_pt 缓存上一采样位置；evaluate(t) 对已生成的多项式求位置，不执行新的规划。
          Eigen::Vector3d last_pt = gl_traj.evaluate(0);
          for (t = 0; t < time; t += ts)
          {
            // evaluate(t)：将从整条候选轨迹起点计时的 t（s）代入已有 XYZ 多项式，返回计划位置（m），不是实际里程计位置。
            Eigen::Vector3d pt = gl_traj.evaluate(t);
            // 当前段长超过目标空间尺度的 1.5 倍（默认 0.3 m），说明取样太疏，放弃这轮并用更小 ts 重采。
            if ((last_pt - pt).norm() > pp_.ctrl_pt_dist * 1.5)
            {
              flag_too_far = true;
              break;
            }
            // 通过间距检查后保存这个采样点，并更新比较基准；point_set 是候选点，不是最终控制点。
            last_pt = pt;
            // vector::push_back(pt) 在列表末尾追加一个位置点，使 size() 增加 1，已有元素保留。
            point_set.push_back(pt);
          }
        // 必须同时满足“相邻样本不过远、点数至少 7”才退出；7 是本初始化的最低采样数策略，不是所有 B 样条的定义要求。
        } while (flag_too_far || point_set.size() < 7); // To make sure the initial path has enough points.
        // for 循环最后又增加了一次 ts，因此减回一次后才是最后一个已保存样本的相对时刻，通常小于候选总时长 time。
        t -= ts;
        // 为后续样条参数化保存四项边界导数；速度单位 m/s、加速度 m/s²，顺序不能交换。
        // 此处终点速度取指定 local_target_vel，终点加速度取最后采样时刻 t 的多项式值，并非一定取 time 处的零加速度。
        start_end_derivatives.push_back(gl_traj.evaluateVel(0));
        start_end_derivatives.push_back(local_target_vel);
        start_end_derivatives.push_back(gl_traj.evaluateAcc(0));
        start_end_derivatives.push_back(gl_traj.evaluateAcc(t));
      }
      else // Initial path generated from previous trajectory.
      {

        // 旧轨迹策略：先取尚未执行的剩余段，必要时补一段多项式连到新局部目标，再按空间距离重采样。
        double t;
        // t_cur 为旧局部轨迹当前相对时间（s）；此分支自身未将它夹紧到 [0,duration_]。
        double t_cur = (ros::Time::now() - local_data_.start_time_).toSec();

        // segment_point 存拼接路线的 XYZ 点；pseudo_arc_length 存相应点距首点的三维折线累计长度（m）。
        // 名称 pseudo 表示由离散样本近似的弧长，不是对连续曲线精确积分；这里含 Z，与前面高度处理的 XY 弧长不同。
        // 例如相邻点间距依次为 0.2、0.3、0.1 m，则该列表为 [0,0.2,0.5,0.6] m，第一项表示首样本处路程为零。
        // 用途：将“想沿候选路线前进多少米”映射到两个相邻样本之间，供后续按近似等空间间距插值取点。
        vector<double> pseudo_arc_length;
        vector<Eigen::Vector3d> segment_point;
        pseudo_arc_length.push_back(0.0);
        // 从当前进度向旧轨迹末端取样；1e-3 s 是末端循环判定的小容差，不是额外延长执行轨迹。
        for (t = t_cur; t < local_data_.duration_ + 1e-3; t += ts)
        {
          segment_point.push_back(local_data_.position_traj_.evaluateDeBoorT(t));
          // 首个样本对应累计长度 0；从第二个样本起，逐段累加三维距离。
          if (t > t_cur)
          {
            pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
          }
        }
        // 回到旧轨迹最后取样时刻，下面将从该处接一段候选；尚未切换到补接多项式自己的时间轴。
        t -= ts;

        // poly_time 是补接段时长估计（s）：旧段末样本距局部目标的直线距离/名义速度，再乘经验放宽系数 2。
        // 很短的补接段（估计时长不大于 ts）跳过此多项式采样，后面的距离重采样仍会显式追加局部目标。
        double poly_time = (local_data_.position_traj_.evaluateDeBoorT(t) - local_target_pt).norm() / pp_.max_vel_ * 2;
        if (poly_time > ts)
        {
          // 用旧轨迹末样本的位置、速度、加速度作起始条件，连接到局部目标状态；候选拼接优先保持运动衔接。
          PolynomialTraj gl_traj = PolynomialTraj::one_segment_traj_gen(local_data_.position_traj_.evaluateDeBoorT(t),
                                                                        local_data_.velocity_traj_.evaluateDeBoorT(t),
                                                                        local_data_.acceleration_traj_.evaluateDeBoorT(t),
                                                                        local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), poly_time);

          // 此处 t 改为补接多项式从 0 开始的相对时间；从 ts 起采样，避免重复添加它与旧段共用的起点。
          for (t = ts; t < poly_time; t += ts)
          {
            if (!pseudo_arc_length.empty())
            {
              // 新点继续接在旧段样本后，累计弧长也从旧段末尾继续增长。
              segment_point.push_back(gl_traj.evaluate(t));
              pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
            }
            else
            {
              ROS_ERROR("pseudo_arc_length is empty, return!");
              continuous_failures_count_++;
              return false;
            }
          }
        }

        // 将拼接折线改为近似等空间间距的样本：sample_length 是下一个样本的目标累计路程（m）。
        double sample_length = 0;
        // cps_dist 在这里是候选采样的空间步长（m），还不是优化后控制点的实际间距。
        // 先乘 1.5 再除回，使第一轮用 pp_.ctrl_pt_dist；点数不足时逐轮缩小步长、增加样本。
        double cps_dist = pp_.ctrl_pt_dist * 1.5; // cps_dist will be divided by 1.5 in the next
        // id 指向当前查找的折线区间 [id,id+1]，不是控制点编号。
        size_t id = 0;
        do
        {
          cps_dist /= 1.5;
          point_set.clear();
          sample_length = 0;
          id = 0;
          // 查找目标累计路程 sample_length 落在哪两个原始样本之间。
          while ((id <= pseudo_arc_length.size() - 2) && sample_length <= pseudo_arc_length.back())
          {
            if (sample_length >= pseudo_arc_length[id] && sample_length < pseudo_arc_length[id + 1])
            {
              // 线性插值：令 s=sample_length，a=(s-s_id)/(s_(id+1)-s_id)，则 P=(1-a)*P_id+a*P_(id+1)。
              // 例如目标路程在 1 m 与 3 m 的样本正中（s=2 m），就取两点坐标的平均值。
              point_set.push_back((sample_length - pseudo_arc_length[id]) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id + 1] +
                                  (pseudo_arc_length[id + 1] - sample_length) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id]);
              // 当前样本找到后，沿折线再前进一个目标空间步长；仍可在同一段内找到多个样本。
              sample_length += cps_dist;
            }
            else
              // 当前区间未包含目标累计路程，继续查看后一个区间。
              id++;
          }
          // 显式保留局部目标，补接最后一段；不能仅因等距离取样没有恰好命中终点，就把目标丢掉。
          point_set.push_back(local_target_pt);
        // 少于 7 点就减小 cps_dist 后从头重采；这里调整空间间距，与上面多项式分支缩小时间间隔 ts 不同。
        } while (point_set.size() < 7); // If the start point is very close to end point, this will help

        // 初值候选来自旧段和补接段，边界导数仍按 [起点速度,终点速度,起点加速度,终点加速度] 保存。
        // 起点导数采自旧轨迹当前进度；终点速度按局部目标指定，终点加速度取零。
        start_end_derivatives.push_back(local_data_.velocity_traj_.evaluateDeBoorT(t_cur));
        start_end_derivatives.push_back(local_target_vel);
        start_end_derivatives.push_back(local_data_.acceleration_traj_.evaluateDeBoorT(t_cur));
        start_end_derivatives.push_back(Eigen::Vector3d::Zero());

        // 期望点数尺度约为“局部范围/目标间距”，乘 3 为经验上限；默认阈值 3.5/0.2*3=52.5，即 53 点起触发。
        // 过多样本可能意味着旧候选绕行太长或取样过密，因此放弃旧段初值，重新从当前起点生成多项式候选。
        if (point_set.size() > pp_.planning_horizon_ / pp_.ctrl_pt_dist * 3) // The initial path is abnormally too long!
        {
          // force=true 让下一轮进入多项式分支，regenerate=true 让本次调用立即再跑一轮初始化。
          flag_force_polynomial = true;
          flag_regenerate = true;
        }
      }
    // 只有要求重新生成时才回到外层开头；成功得到一组候选点和边界导数后，进入后续高度处理和样条参数化。
    } while (flag_regenerate);

    // 在样条参数化前统一生成参考高度，后续优化器把 z 梯度置零，二者共同限定当前高度自由度。
    applyLinearZReference(point_set, start_pt(2), local_target_pt(2));

    // 优化真正使用的“初值”是下面求出的 ctrl_pts（3×N，每列为一个控制点的 XYZ），不是多项式的常数项或 t=0 的位置。
    // point_set 是候选曲线的采样点；结合首尾导数约束求控制点，两者不是同一组点，也不是直接逐点复制。
    Eigen::MatrixXd ctrl_pts;
    UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);

    vector<vector<Eigen::Vector3d>> a_star_paths;
    // 局部规划仍以 B 样条优化为主体；initControlPoints() 检测初始候选的碰撞区段，必要时用 A* 搜索绕障引导。
    // 该函数一边将引导转换为优化器内部的障碍基点/推离方向，供后续碰撞代价使用，一边返回路径列表给这里显示。
    // 因此下面的 a_star_paths 主要交给 displayAStarList()；真正影响优化的引导已保存在优化器内，不需再次作为参数传入。
    // 顺序是“先有初始 B 样条控制点 → A* 辅助构造避障方向 → 优化这些控制点”，最终发布的是优化后的样条。
    a_star_paths = bspline_optimizer_rebound_->initControlPoints(ctrl_pts, true);

    // 初始化耗时：候选生成/采样、高度处理、控制点参数化与 A* 碰撞引导构造。
    t_init = ros::Time::now() - t_start;

    static int vis_id = 0;
    visualization_->displayInitPathList(point_set, 0.2, 0);
    visualization_->displayAStarList(a_star_paths, vis_id);

    t_start = ros::Time::now();

    /*** STEP 2: OPTIMIZE ***/
    // 以这组初始控制点为起点迭代优化可调控制点，综合避障、平滑和运动限值代价，成功后 ctrl_pts 保存优化结果。
    bool flag_step_1_success = bspline_optimizer_rebound_->BsplineOptimizeTrajRebound(ctrl_pts, ts);
    cout << "first_optimize_step_success=" << flag_step_1_success << endl;
    if (!flag_step_1_success)
    {
      // visualization_->displayOptimalList( ctrl_pts, vis_id );
      continuous_failures_count_++;
      return false;
    }
    //visualization_->displayOptimalList( ctrl_pts, vis_id );

    // Rebound 控制点优化阶段的耗时。
    t_opt = ros::Time::now() - t_start;
    t_start = ros::Time::now();

    /*** STEP 3: REFINE(RE-ALLOCATE TIME) IF NECESSARY ***/
    // 前一步优化使用平滑/避障/运动限值代价，代价惩罚不等于已严格满足限值；这里检查时间安排是否让运动过快。
    // 目标是“给已有路线更多执行时间，必要时重新调整控制点”，不是等待一段时间，也不是重新规划整条导航任务。
    // 理想的整体时间放大 k 倍时，速度降至 1/k、加速度降至 1/k²；实际节点拉伸后还需重新参数化、精修和检查。
    // 由优化后的控制点、次数 3 和节点间隔 ts，构造可求位置/导数的三次 B 样条对象。
    UniformBspline pos = UniformBspline(ctrl_pts, 3, ts);
    pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_, pp_.feasibility_tolerance_);

    // ratio 由 checkFeasibility() 输出，是根据导数超限程度估计的时间放大尺度，无单位；并非直接将整条曲线每个时刻精确乘此值。
    double ratio;
    // 默认认为本阶段成功；只有需要精修时，才用精修结果覆盖此标志；因此检查通过时无需做额外精修。
    bool flag_step_2_success = true;
    // 导数控制点检查不通过时拉长时间并精修；即使通过，下面仍会做范数采样检查。
    if (!pos.checkFeasibility(ratio, false))
    {
      cout << "Need to reallocate time." << endl;

      Eigen::MatrixXd optimal_control_points;
      // 内部依次拉伸节点时间、重采样并按首尾导数求新控制点，再以平滑/参考贴合/可行性代价精修；ts 按引用更新。
      flag_step_2_success = refineTrajAlgo(pos, start_end_derivatives, ratio, ts, optimal_control_points);
      if (flag_step_2_success)
        // 使用精修后的控制点和更新的 ts 替换待保存的轨迹，而不是继续使用原始时间安排。
        pos = UniformBspline(optimal_control_points, 3, ts);
    }

    // 精修失败或最终动态检查失败时返回 false，不更新本次可发布轨迹。
    // checkDynamicFeasibility() 逐时刻采样速度/加速度范数，按名义限值+加性容差检查；它本身不做地图碰撞检查。
    if (!flag_step_2_success || !checkDynamicFeasibility(pos))
    {
      printf("\033[34mThis refined trajectory is unsafe or dynamically infeasible. Skip publishing it.\n\033[0m");
      continuous_failures_count_++;
      return false;
    }

    // 可行性检查以及必要的时间调整/精修耗时；即使无需精修，也包含检查花费的时间。
    t_refine = ros::Time::now() - t_start;

    // save planned results
    updateTrajInfo(pos, ros::Time::now());

    cout << "total time:\033[42m" << (t_init + t_opt + t_refine).toSec() << "\033[0m,optimize:" << (t_init + t_opt).toSec() << ",refine:" << t_refine.toSec() << endl;

    // success. YoY
    continuous_failures_count_ = 0;
    return true;
  }

  /// @brief 以同一位置重复控制点构造停止 B 样条，保存为新的局部轨迹。
  bool SCANPlannerManager::EmergencyStop(Eigen::Vector3d stop_pos)
  {
    Eigen::MatrixXd control_points(3, 6);
    for (int i = 0; i < 6; i++)
    {
      control_points.col(i) = stop_pos;
    }

    updateTrajInfo(UniformBspline(control_points, 3, 1.0), ros::Time::now());

    return true;
  }

  /// @brief 把当前状态和给定路径点转换为分段多项式参考，距离过大时插入中间点。
  /// 此函数没有执行全局障碍物搜索，参考的可用性还需后续局部规划判断。
  bool SCANPlannerManager::planGlobalTrajWaypoints(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                                  const std::vector<Eigen::Vector3d> &waypoints, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {

    // generate global reference trajectory

    if (waypoints.empty())
      return false;

    vector<Eigen::Vector3d> points;
    points.push_back(start_pos);

    for (size_t wp_i = 0; wp_i < waypoints.size(); wp_i++)
    {
      points.push_back(waypoints[wp_i]);
    }

    double total_len = 0;
    for (size_t i = 0; i < points.size() - 1; i++)
    {
      total_len += (points[i + 1] - points[i]).norm();
    }

    // insert intermediate points if too far
    vector<Eigen::Vector3d> inter_points;
    double dist_thresh = max(total_len / 8, 4.0);

    for (size_t i = 0; i < points.size() - 1; ++i)
    {
      inter_points.push_back(points.at(i));
      double dist = (points.at(i + 1) - points.at(i)).norm();

      if (dist > dist_thresh)
      {
        int id_num = floor(dist / dist_thresh) + 1;

        for (int j = 1; j < id_num; ++j)
        {
          Eigen::Vector3d inter_pt =
              points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
          inter_points.push_back(inter_pt);
        }
      }
    }

    inter_points.push_back(points.back());

    // for ( int i=0; i<inter_points.size(); i++ )
    // {
    //   cout << inter_points[i].transpose() << endl;
    // }

    // write position matrix
    int pt_num = inter_points.size();
    Eigen::MatrixXd pos(3, pt_num);
    for (int i = 0; i < pt_num; ++i)
      pos.col(i) = inter_points[i];

    Eigen::Vector3d zero(0, 0, 0);
    // mode 3 传入的是空间点列表，未携带外部各点时序；这里按点间距离和本机速度配置重新分配参考段时长。
    Eigen::VectorXd time(pt_num - 1);
    for (int i = 0; i < pt_num - 1; ++i)
    {
      time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
    }

    time(0) *= 2.0;
    time(time.rows() - 1) *= 2.0;

    PolynomialTraj gl_traj;
    if (pos.cols() >= 3)
      gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
    else if (pos.cols() == 2)
      gl_traj = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, pos.col(1), end_vel, end_acc, time(0));
    else
      return false;

    auto time_now = ros::Time::now();
    global_data_.setGlobalTraj(gl_traj, time_now);

    return true;
  }

  /// @brief 生成起点至终点的多项式参考并存入 global_data_，边界速度和加速度由调用方提供。
  // 全局在这里表示覆盖当前起终点的参考范围；本函数不查询障碍地图或做全局 A*，因此参考可能穿过障碍。
  // 两点用边界条件求单段五次多项式，插入中间点后求分段多项式；后续局部规划承担地图避障与可行性优化。
  // 原理是“给定点和时间后的连续曲线生成”：首尾位置、速度、加速度形成方程约束，求 XYZ 多项式系数。
  // 与 A* 的任务不同：A* 在地图上选择可通行的空间路线；本函数按输入点构造参考，不决定从障碍哪侧绕行。
  // mode 1/2 的两点参考主要表达任务方向；mode 3 则将外部给出的路线点串成参考，后续 FSM 沿参考选局部目标。
  // 调用实参按顺序绑定到这六个形参；第 3 个是 start_acc，第 5 个是 end_vel，第 6 个是 end_acc。
  // 三者都为 (0,0,0) 时依然是三个不同角色的输入；下方多项式生成器分别将它们用于对应首尾导数约束。
  bool SCANPlannerManager::planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                         const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {

    // generate global reference trajectory

    vector<Eigen::Vector3d> points;
    points.push_back(start_pos);
    points.push_back(end_pos);

    // insert intermediate points if too far
    // 在起终点连线上插入几何中间点，控制单段跨度；这里只做线性插值，不读取障碍，因此不是在搜索绕障关键点。
    // 该参考生成不替代全局 A*；需要全局绕行时可另接上层路线搜索，不能因多项式光滑就声称已经全局避障。
    // 面试辨析：全局参考生成与地图路线搜索分工不同，见 docs/面试/算法原理与面试问题.md#全局搜索与多项式参考。
    vector<Eigen::Vector3d> inter_points;
    const double dist_thresh = 4.0;

    for (size_t i = 0; i < points.size() - 1; ++i)
    {
      inter_points.push_back(points.at(i));
      double dist = (points.at(i + 1) - points.at(i)).norm();

      if (dist > dist_thresh)
      {
        int id_num = floor(dist / dist_thresh) + 1;

        for (int j = 1; j < id_num; ++j)
        {
          Eigen::Vector3d inter_pt =
              points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
          inter_points.push_back(inter_pt);
        }
      }
    }

    inter_points.push_back(points.back());

    // write position matrix
    int pt_num = inter_points.size();
    Eigen::MatrixXd pos(3, pt_num);
    for (int i = 0; i < pt_num; ++i)
      pos.col(i) = inter_points[i];

    Eigen::Vector3d zero(0, 0, 0);
    // 先以段长/名义速度给定各段时间，再加长首尾段；之后多项式求解使用这些固定时长，不在此做最短时间搜索。
    Eigen::VectorXd time(pt_num - 1);
    for (int i = 0; i < pt_num - 1; ++i)
    {
      time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
    }

    time(0) *= 2.0;
    time(time.rows() - 1) *= 2.0;

    PolynomialTraj gl_traj;
    if (pos.cols() >= 3)
      gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
    else if (pos.cols() == 2)
      gl_traj = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, time(0));
    else
      return false;

    auto time_now = ros::Time::now();
    global_data_.setGlobalTraj(gl_traj, time_now);

    return true;
  }

  /// @brief 先按可行性比例拉长轨迹时间并重新参数化，再以旧形状采样点为参考进行 refine 优化。
  bool SCANPlannerManager::refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points)
  {
    double t_inc;

    Eigen::MatrixXd ctrl_pts; // = traj.getControlPoint()

    // std::cout << "ratio: " << ratio << std::endl;
    reparamBspline(traj, start_end_derivative, ratio, ctrl_pts, ts, t_inc);

    traj = UniformBspline(ctrl_pts, 3, ts);

    double t_step = traj.getTimeSum() / (ctrl_pts.cols() - 3);
    bspline_optimizer_rebound_->ref_pts_.clear();
    for (double t = 0; t < traj.getTimeSum() + 1e-4; t += t_step)
      bspline_optimizer_rebound_->ref_pts_.push_back(traj.evaluateDeBoorT(t));

    bool success = bspline_optimizer_rebound_->BsplineOptimizeTrajRefine(ctrl_pts, ts, optimal_control_points);

    return success;
  }

  /// @brief 保存位置样条并求导得到速度、加速度样条，统一更新时间、时长、起点和轨迹编号。
  void SCANPlannerManager::updateTrajInfo(const UniformBspline &position_traj, const ros::Time time_now)
  {
    local_data_.start_time_ = time_now;
    local_data_.position_traj_ = position_traj;
    local_data_.velocity_traj_ = local_data_.position_traj_.getDerivative();
    local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();
    local_data_.start_pos_ = local_data_.position_traj_.evaluateDeBoorT(0.0);
    local_data_.duration_ = local_data_.position_traj_.getTimeSum();
    local_data_.traj_id_ += 1;
  }

  /// @brief 以 0.01~0.05 s 的时间步采样速度与加速度范数，使用名义上限加绝对容差判断。
  /// 这里只检查运动学可行性，不执行占用地图碰撞检查。
  bool SCANPlannerManager::checkDynamicFeasibility(UniformBspline position_traj)
  {
    UniformBspline vel_traj = position_traj.getDerivative();
    UniformBspline acc_traj = vel_traj.getDerivative();
    const double duration = position_traj.getTimeSum();
    const double sample_dt = std::max(0.01, std::min(0.05, duration / 50.0));
    const double vel_limit = pp_.max_vel_ + pp_.vel_tolerance_;
    const double acc_limit = pp_.max_acc_ + pp_.acc_tolerance_;

    for (double t = 0.0; t < duration + 1e-6; t += sample_dt)
    {
      const double tc = std::min(t, duration);
      Eigen::Vector3d vel = vel_traj.evaluateDeBoorT(tc);
      if (vel.norm() > vel_limit)
      {
        // ROS_WARN_STREAM 是警告级日志宏，与 ROS_WARN 同级；STREAM 允许用 << 拼接文本和数值，无需 %f 格式符。
        // 记录超限采样时刻 tc（s）、速度范数及门槛（m/s）；该宏不自带限频或停止功能，下面的 return false 才报告检查失败。
        ROS_WARN_STREAM("Dynamic feasibility check failed: velocity limit exceeded at t="
                        << tc << ", |v|=" << vel.norm() << " > " << vel_limit);
        return false;
      }

      Eigen::Vector3d acc = acc_traj.evaluateDeBoorT(tc);
      if (acc.norm() > acc_limit)
      {
        // 同样采用流式警告日志，输出加速度范数与门槛（m/s²）；STREAM 改变消息写法，不改变日志严重级别。
        ROS_WARN_STREAM("Dynamic feasibility check failed: acceleration limit exceeded at t="
                        << tc << ", |a|=" << acc.norm() << " > " << acc_limit);
        return false;
      }
    }

    return true;
  }

  /// @brief 拉长时间后重采样轨迹，结合首尾导数重新求控制点，返回新的时间间隔和时间增量。
  // 用于 REFINE：已有路线因速度/加速度检查不通过而需要放慢，重建适合后续均匀样条精修的控制点与节点间隔。
  // bspline：输入已有样条，lengthenTime() 会原地修改它的节点时间；不在此直接发布轨迹。
  // start_end_derivative：输入 [起点速度,终点速度,起点加速度,终点加速度]，本函数只读取它，用于保留衔接要求。
  // ratio：输入时间拉伸尺度，无单位；ctrl_pts：输出重新求得的 XYZ 控制点矩阵。
  // dt/time_inc：引用输出，分别是新的均匀节点间隔和增加的总时长（s）；调用者传入的 ts/t_inc 会被写回。
  void SCANPlannerManager::reparamBspline(UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative, double ratio,
                                         Eigen::MatrixXd &ctrl_pts, double &dt, double &time_inc)
  {
    // 记录拉伸前的执行总时长（s），供后面计算增加了多少时间；不是当前 ROS 时刻。
    double time_origin = bspline.getTimeSum();
    // 本流程使用三次 B 样条：N 个控制点对应 N-3 个有效时间区间（每区间由四个相邻控制点支撑）。
    // 这里保持区间数，用新的总时长重新计算均匀间隔；不是将 N 个控制点用直线连成 N-1 段。
    int seg_num = bspline.getControlPoint().cols() - 3;
    // double length = bspline.getLength(0.1);
    // int seg_num = ceil(length / pp_.ctrl_pt_dist);

    // 拉伸内部节点时间，给部分运动更多时间；该实现不把所有节点统一乘 ratio，节点间隔可能不再均匀。
    // 因此应读回实际新时长，而不能直接设 duration=time_origin*ratio；非均匀节点变化也可能改变局部几何形状。
    bspline.lengthenTime(ratio);
    double duration = bspline.getTimeSum();
    // 用实际新时长除以原区间数，得到重建均匀样条时每个时间区间的长度（s）。
    dt = duration / double(seg_num);
    time_inc = duration - time_origin;

    // 对拉伸后的已有样条重新采位置点；这些点用于拟合/参数化，区别于下一步要求解的控制点。
    vector<Eigen::Vector3d> point_set;
    // 计算 t=0、dt、2dt……到新末时刻的位置；1e-4 s 是浮点端点容差，本循环不等待真实时间流逝。
    for (double time = 0.0; time <= duration + 1e-4; time += dt)
    {
      point_set.push_back(bspline.evaluateDeBoorT(time));
    }
    // 同时考虑新位置样本和原首尾导数要求，以 dt 为均匀时间间隔重新求控制点。
    // 若仅整体放大时间，非零首尾速度/加速度也会缩小，可能与旧轨迹/实际运动衔接不符，因此这里重新加入导数条件。
    // 该参数化用 QR 解超定系统作最小二乘拟合，不保证每个采样点和所有导数条件同时精确满足，后续仍需精修与检查。
    UniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, ctrl_pts);
  }

} // namespace scan_planner
