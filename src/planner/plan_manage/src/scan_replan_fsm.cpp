
/// @file
/// 规划状态机：由 scan_planner_node 的 main() 创建，管理目标、重规划、轨迹发布和恢复。
/// 输入为机身 Odometry、所选导航模式的目标和控制器进度冻结状态；输出为 Bspline 和显示 Marker。
/// 位置单位 m，时间单位 s；状态定时器目标周期 0.01 s，安全定时器 0.05 s。
/// 回调由入口的 ros::spin() 调度；定时器周期不等于新轨迹生成频率。
#include <plan_manage/scan_replan_fsm.h>
// 本文件的数值处理：floor/ceil 计算参考采样数量，atan2/cos/sin 计算水平朝向及机身几何位置。
#include <cmath>
// 本文件在 mode 2 使用 std::system() 执行 rosparam load，从关键点 YAML 加载任务配置。
#include <cstdlib>

// 匿名命名空间：把 shellQuote 作为当前编译单元的内部工具，不暴露为 scan_planner 的公共接口。
// 各 cpp 的匿名命名空间彼此独立，其他源文件即使有同名辅助函数也不会产生跨文件的同名定义冲突。
// namespace 只组织名字和链接范围，不会自动执行其中函数；实际调用在下面 init() 的 mode 2 分支。
namespace
{
  // 为 rosparam load 的目标命名空间构造一个 Shell 字符串参数，避免空格或单引号改变参数划分。
  // 本函数只返回命令字符串的一部分，不执行命令、不改变 ROS 参数或节点命名空间。
  //
  // 阅读时分两层：先由 C++ 编译器把源码字面量变成字符串字符，之后 Shell 才解释这些字符。
  // C++ 中：双引号包围字符串，单引号包围一个字符；两侧的边界引号不存入内容。
  // 反斜杠用于引出转义："\n" 存一个换行符，"\\" 存一个反斜杠，"\"" 存一个双引号。
  // 所以源码中的两个反斜杠不是运行时再被删除一个，而是编译时共同表示一个反斜杠字符。
  //
  // 示例（下面直接展示输入/返回值的字符内容，不是它们的 C++ 源码字面量）：
  //   输入：abc                 返回：'abc'
  //   输入：a'b                 返回：'a'\''b'
  //   输入：hello world         返回：'hello world'
  // 第二例：Shell 将 'a'、\'、'b' 三个相邻片段拼成同一个参数 a'b；空格同样保留在一个参数内。
  std::string shellQuote(const std::string &value)
  {
    // 这里源码外侧的 "..." 是 C++ 字符串边界，内部 ' 是普通内容，因此初始 quoted 只有一个单引号。
    // 这个单引号先存入字符串，之后用于打开 Shell 的单引号段。
    std::string quoted = "'";
    // 依次取出 value 的每个字符；c 是本轮只读的字符副本，quoted 是逐步累加的结果。
    for (const char c : value)
    {
      // 字符字面量 '\''：外侧两个 ' 是 C++ 边界，内部 \' 表示一个单引号；判断 c 是否正好为它。
      if (c == '\'')
        // += 表示追加到已有 quoted 后面。下面 "..." 中的 \\ 被 C++ 解释为一个反斜杠。
        // 实际追加 4 个字符，按顺序是 [单引号][反斜杠][单引号][单引号]，内容为：'\''
        // Shell 以后分三步解释：第一个 ' 关闭已有引号段；中间 \' 插入单引号；最后 ' 重新打开引号段。
        quoted += "'\\''";
      else
        // 普通字符原样追加，包括空格；它们位于生成的 Shell 单引号段内部。
        quoted += c;
    }
    // 补齐 Shell 参数末尾的单引号。例 a'b 的累加过程（每行均为实际字符串内容）：
    //   初始：'  → 处理 a 后：'a  → 处理输入单引号后：'a'\''  → 处理 b 后：'a'\''b
    //   执行下句后：'a'\''b'，最终返回该字符串。
    quoted += "'";
    return quoted;
  }
} // namespace

// 项目的公开命名空间：这里实现 scan_replan_fsm.h 中声明的 SCANReplanFSM 成员。
// 下面的实现可直接调用本文件匿名命名空间的 shellQuote；它仍是文件内部工具，并非类成员。
namespace scan_planner
{

  /// @brief 读取私有参数并初始化模块和回调，最后按 navi_mode 选择目标入口。
  /// mode 2 在这里运行 rosparam load；关键点文件缺失或点数无效会关闭节点。
  // 三种模式的函数调用图见仓库 README.md 的“三种导航模式的函数调用图”（scan-study-mode-flow）。
  // 各模式只分开准备任务入口，共用后续执行状态机、安全检查和局部轨迹发布；回调注册不代表立即顺序执行。
  void SCANReplanFSM::init(ros::NodeHandle &nh)
  {
    // 新任务先指向第 0 个关键点；只有 mode 2 实际使用关键点序列推进。
    current_wp_ = 0;
    // 启动处于等待输入阶段，后续由目标、位姿和定时回调推进状态。
    // 在 init() 中显式设置初始枚举状态 INIT（数值为 0），不是给 exec_state_ 赋字符串 "INIT"。
    exec_state_ = FSM_EXEC_STATE::INIT;
    // 尚无导航触发，避免节点一启动就自动生成轨迹。
    trigger_ = false;
    // 尚未构造出有效目标参考，等待目标入口设置。
    have_target_ = false;
    // 尚无机身位置/速度反馈，不能确定有效规划起点。
    have_odom_ = false;
    // 尚无新目标；目标更新后置 true 以要求本次规划重新构造初值。
    have_new_target_ = false;
    // mode 1 的目标高度尚未从首条机身位姿取得，在此之前拒绝点击目标。
    rviz_height_ready_ = false;
    // 启动时未收到“原地转向冻结”反馈，轨迹时间无需补偿。
    go2_execution_frozen_ = false;
    // 准备允许首次进入急停时发布一条停止轨迹，避免每次安全回调反复发布同一停止轨迹。
    flag_escape_emergency_ = true;
    // 尚未因失败达到上限而要求停车等待新目标；后续失败处理可置 true。
    need_hover_stop_ = false;
    // 清空状态机失败次数；普通规划成功时也会清零，达到上限时进入停车处理。
    replan_fail_count_ = 0;
    // 记录相邻冻结更新调用的时间基准；后续每次调用都会刷新，与是否冻结无关。
    last_freeze_update_time_ = ros::Time::now();

    /*  fsm param  */
    // 本段读取启动配置：下方“正常默认”来自当前 launch/XML，实际运行以参数服务器为准。
    // param() 的末项仅是缺参回退；getParam()/必需参数封装没有这一回退，具体见各行说明。
    // 参数读取：导航模式，整数 1/2/3；决定 RViz 目标、关键点文件或参考路径的输入入口。
    // 配置来源：advanced_param.xml → /scan_planner_node/fsm/navi_mode；正常默认启动取 1。
    // 缺参回退：-1；只有参数服务器中没有该键时才使用。
    nh.param("fsm/navi_mode", navi_mode_, -1);
    // 参数读取：常规重规划的起点距离阈值，单位 m。EXEC_TRAJ 中，当前时刻轨迹采样位置到本条轨迹起点
    // 的欧氏距离小于该值时，暂不常规重规划。
    // 配置来源：advanced_param.xml → /scan_planner_node/fsm/thresh_replan；正常默认启动取 1.0。
    // 缺参回退：-1.0；只有参数服务器中没有该键时才使用。
    nh.param("fsm/thresh_replan", replan_thresh_, -1.0);
    // 参数读取：当前轨迹采样位置距最终目标小于该值时，暂不常规重规划；单位 m，不是控制器停车误差阈
    // 值。
    // 配置来源：advanced_param.xml → /scan_planner_node/fsm/thresh_no_replan；正常默认启动取 0.1。
    // 缺参回退：-1.0；只有参数服务器中没有该键时才使用。
    nh.param("fsm/thresh_no_replan", no_replan_thresh_, -1.0);
    // 参数读取：getLocalTarget() 沿全局参考轨迹累计的前视距离，单位 m；参考剩余距离较短时使用终点。
    // 配置来源：advanced_param.xml → /scan_planner_node/fsm/planning_horizon；正常默认启动取 3.5。
    // 缺参回退：-1.0；只有参数服务器中没有该键时才使用。
    nh.param("fsm/planning_horizon", planning_horizon_, -1.0);
    // 参数读取：急停时间阈值，单位 s；参数名末尾的下划线是现有接口的一部分。安全回调发现碰撞风险且
    // 立即重规划失败后，若预测碰撞距当前不足该时间，则进入 EMERGENCY_STOP。
    // 配置来源：advanced_param.xml → /scan_planner_node/fsm/emergency_time_；正常默认启动取 1.0。
    // 缺参回退：1.0；只有参数服务器中没有该键时才使用。
    nh.param("fsm/emergency_time_", emergency_time_, 1.0);
    // 参数读取：急停后的自动恢复开关，不是碰撞检测总开关。true：速度降到 0.1 m/s 以下后，按
    // need_hover_stop_ 决定重规划或等待新目标；false 不自动退出急停。
    // 配置来源：advanced_param.xml → /scan_planner_node/fsm/fail_safe；正常默认启动取 true。
    // 缺参回退：true；只有参数服务器中没有该键时才使用。
    nh.param("fsm/fail_safe", enable_fail_safe_, true);
    // 参数读取：普通生成/重规划失败的次数上限，达到后停车并等待新目标；不是秒数或优化器内部迭代上限
    // 。
    // 配置来源：advanced_param.xml → /scan_planner_node/fsm/max_replan_fail_count；正常默认启动取
    // 1000。
    // 缺参回退：1000；只有参数服务器中没有该键时才使用。
    nh.param("fsm/max_replan_fail_count", max_replan_fail_count_, 1000);
    // 参数读取：本状态机的圆柱显示上厚度（m），用于 publishSelfInflationMarker()；实际地图膨胀由
    // GridMap 另读同名键维护。
    // 配置来源：advanced_param.xml → /scan_planner_node/grid_map/obstacles_inflation_z_up；正常默认
    // 启动取 0.1。
    // 缺参回退：0.0；只有参数服务器中没有该键时才使用。
    nh.param("grid_map/obstacles_inflation_z_up", self_inflation_z_up_, 0.0);
    // 参数读取：本状态机的圆柱显示下厚度（m），与上厚度共同决定显示高度；不在此处更新地图占用。
    // 配置来源：advanced_param.xml → /scan_planner_node/grid_map/obstacles_inflation_z_down；正常默
    // 认启动取 0.1。
    // 缺参回退：0.0；只有参数服务器中没有该键时才使用。
    nh.param("grid_map/obstacles_inflation_z_down", self_inflation_z_down_, 0.0);
    // 参数读取：本状态机的圆柱显示半径（m），默认 0.25；实际碰撞查询半径由 GridMap 的独立缓存使用。
    // 配置来源：advanced_param.xml → /scan_planner_node/grid_map/double_cylinder_radius；正常默认启
    // 动取 0.25。
    // 缺参回退：0.0；只有参数服务器中没有该键时才使用。
    nh.param("grid_map/double_cylinder_radius", self_double_cylinder_radius_, 0.0);
    // 参数读取：本状态机前后显示圆柱的中心偏移（m），默认 0.18，两中心间距 0.36，随机身 yaw 排列。
    // 配置来源：advanced_param.xml → /scan_planner_node/grid_map/double_cylinder_offset；正常默认启
    // 动取 0.18。
    // 缺参回退：0.0；只有参数服务器中没有该键时才使用。
    nh.param("grid_map/double_cylinder_offset", self_double_cylinder_offset_, 0.0);
    // 参数读取：mode 3 的参考路径高度偏移，单位 m；FSM::pathCallback() 将其加到 /initial_path 的 z
    // 上。当前默认 mode 1 的目标 z 来自首次机身位姿；此值不会直接改变圆柱膨胀高度或仿真初始 z。
    // 配置来源：advanced_param.xml → /scan_planner_node/grid_map/body_height；正常默认启动取 0.4。
    // 缺参回退：0.0；只有参数服务器中没有该键时才使用。
    nh.param("grid_map/body_height", body_height_, 0.0);
    // 参数读取：机身圆柱 Marker 的参考坐标系名，本状态机用于 self_inflation 显示；未配置时为 world
    // 。
    // 配置位置：私有 /scan_planner_node/grid_map/frame_id；当前主 launch 未覆盖此键，取下面的缺参回
    // 退。
    // 缺参回退：std::string("world")；只有参数服务器中没有该键时才使用。
    nh.param("grid_map/frame_id", self_inflation_frame_id_, std::string("world"));

    if (navi_mode_ == NAVI_MODE::PRESET_TARGET)
    {
      // 从 scan_planner 包位置回到仓库 tools 目录取得关键点 YAML；文件内容由录制工具生成。
      const std::string keypoints_yaml = "\"$(rospack find scan_planner)/../../../tools/keypoint.yaml\"";
      // 把 YAML 加载命令与当前私有参数命名空间拼接，使下面读取 fsm/waypoint* 时能找到相同位置。
      const std::string load_keypoints_cmd =
          "rosparam load " + keypoints_yaml + " " + shellQuote(nh.getNamespace());
      // load_keypoints_cmd 是 std::string；system() 接收 const char*，因此用 c_str() 取得命令文字的只读字符指针。
      // 指向的字符序列以 '\0' 结束，例如 "hello" 对应 [h,e,l,l,o,\0]；结束标记不是要执行的命令文字。
      // c_str() 不修改原字符串，也不执行命令；实际执行由 std::system() 完成。
      // 可分步理解为：const char* command = load_keypoints_cmd.c_str(); 然后 std::system(command)。
      // 执行加载命令；非零结果表示未成功加载，当前实现关闭节点并提前返回，不继续使用缺失关键点。
      if (std::system(load_keypoints_cmd.c_str()) != 0)
      {
        ROS_ERROR("[SCANReplanFSM] Failed to load keypoints_yaml: tools/keypoint.yaml");
        ros::shutdown();
        return;
      }

      // 参数读取：mode 2 的关键点数量，运行时由 tools/keypoint.yaml 写入；必须为正，当前默认 mode 1
      // 不读取。
      // 配置位置：私有 /scan_planner_node/fsm/waypoint_num；当前主 launch 未覆盖此键，取下面的缺参
      // 回退。
      // 缺参/读取失败时回退 -1：这是本项目选用的值，不是 ROS 规定所有参数必须使用 -1。
      // 对点数而言，-1 无效且会被下面 <=0 的检查捕获，因此不会按负数长度继续建立列表。
      nh.param("fsm/waypoint_num", waypoint_num_, -1);

      // mode 2 必须有正数个点；缺少 waypoint_num 时回退 -1，因此也会进入这个分支。
      if (waypoint_num_ <= 0)
      {
        ROS_ERROR("[SCANReplanFSM] navi_mode=2 requires keypoints_yaml with fsm/waypoint_num and fsm/waypoint{i}_{x,y,z}.");
        ros::shutdown();
        return;
      }
      // preset_waypoints_ 是状态机成员，已在 scan_replan_fsm.h 声明，不需在本成员函数中重复声明。
      // 首次构造时列表为空；resize(N) 将元素个数调整为 N，使 [0] 到 [N-1] 都有三维点对象可访问。
      // resize() 是 std::vector 的成员函数；这一步不读取坐标，也不应假设 Eigen 点的 XYZ 已全部为零。
      // 例如 waypoint_num_=2，调整后有 preset_waypoints_[0] 和 [1]，再由循环逐点填入 XYZ（m）。
      preset_waypoints_.resize(waypoint_num_);
      // i 从 0 递增到 waypoint_num_-1；每轮读取同一个点的 X、Y、Z 三个分量。
      for (int i = 0; i < waypoint_num_; i++)
      {
        // to_string(i) 是标准库的数字转字符串函数：0→"0"、1→"1"，用于拼出参数名，而不是数值相加。
        // i=0 时 X 参数名为 "fsm/waypoint0_x"；i=1 时为 "fsm/waypoint1_x"。
        // [i] 先选列表中第 i 个点，随后 Eigen 的 (0)/(1)/(2) 分别访问该点的 X/Y/Z 分量。
        // 例如 i=0 时，下面三句分别把参数值写入第一个点的 X、Y、Z，不会创建三个不同的点。
        //
        // 三轴的 -1.0 是本代码统一选取的回退数值：缺少键或不能按期望类型读取时写入对应分量。
        // 参数存在且读取成功时使用真实参数值，哪怕该值恰好就是 -1.0；回退值不是“拒绝负坐标”的规则。
        // 本循环没有检查 param() 的成功返回值，也未逐轴验证缺失，因此回退后仍会继续保存并用于后续规划。
        // 这与前面点数 <=0 时关闭节点不同；不能只靠某分量是否等于 -1.0 判断它是否读取失败。
        //
        // 参数读取：第 i 个关键点 X（世界系 m），读取失败回退 -1.0，写入当前点第 0 分量。
        nh.param("fsm/waypoint" + to_string(i) + "_x", preset_waypoints_[i](0), -1.0);
        // 参数读取：第 i 个关键点 Y（世界系 m），读取失败回退 -1.0，写入当前点第 1 分量。
        nh.param("fsm/waypoint" + to_string(i) + "_y", preset_waypoints_[i](1), -1.0);
        // 参数读取：第 i 个关键点 Z（世界系 m），读取失败回退 -1.0，写入当前点第 2 分量。
        nh.param("fsm/waypoint" + to_string(i) + "_z", preset_waypoints_[i](2), -1.0);
      }
      // 结果示例：若两点的参数分别给出 (1,2,0.5)、(3,4,0.5)，列表两个元素就是这两个坐标。
      // 若仅 waypoint0_x 缺失而其他参数相同，则第一个点变为 (-1,2,0.5)，本段不会自动报错退出。
    }

    /* initialize main modules */
    // 创建规划显示入口，后续 FSM 与管理器用它发布目标、参考和局部规划过程 Marker。
    // 从右向左理解：new PlanningVisualization(nh) 创建实际显示对象并立即执行带 nh 的构造函数。
    // reset(...) 是 visualization_ 这个共享智能指针的操作，接管刚创建的对象；首次初始化前指针为空。
    // 这里创建的是 PlanningVisualization 对象，头文件中的 PlanningVisualization::Ptr 只是管理它的成员类型。
    visualization_.reset(new PlanningVisualization(nh));
    // 创建统一规划管理器，接收状态机的轨迹生成/重规划/停止请求。
    // 这一版 new 不传 nh，调用管理器的无参构造；地图、优化器与搜索池在下一句 initPlanModules() 中建立。
    planner_manager_.reset(new SCANPlannerManager);
    // 建立共享占用地图、样条优化器和 A* 节点池，并接入刚创建的显示入口。
    // 传入同一可视化对象的共享指针，使 FSM 与管理器各自持有并复用其发布接口，而不是再创建一套显示对象。
    // 上面的 .reset(...) 操作智能指针自身；这里的 -> 则访问它所指的 SCANPlannerManager 对象。
    // initPlanModules() 是管理器的普通成员函数，显式调用后才读取规划参数并创建算法模块，不会随空构造自动执行。
    // nh 为这些模块提供同一规划节点的参数/通信入口；visualization_ 提供可复用的显示入口。
    planner_manager_->initPlanModules(nh, visualization_);

    /* callback */
    // 定时任务：执行状态机，周期 0.01 s（目标 100 Hz）；仅按状态判断是否需要新规划，不等于每秒生成
    // 100 条轨迹。
    // createTimer() 是 ros::NodeHandle 的成员函数，这里通过 nh 创建定时任务，返回句柄保存在 exec_timer_。
    // 本机 ROS 对应的成员回调模板定义位于 /opt/ros/noetic/include/ros/node_handle.h，
    // 在 "Versions of createTimer()" 一节；它有多个重载，本次选择“成员函数 + 对象指针”这一版。
    // 调用参数在这里分别指定：
    //   ros::Duration(0.01)：两次触发的目标间隔，单位 s。
    //   &SCANReplanFSM::execFSMCallback：选择将来执行的成员函数，不是在此处立即调用它。
    //   this：选择在哪个状态机对象上执行；当前处于 SCANReplanFSM::init()，它指向 main() 创建的 scan_replan。
    // 成员函数会访问对象自己的 exec_state_、目标和位姿，因此 ROS 必须同时保存“函数”和“对象”。
    // & 在此处取得成员函数指针，与形参里的引用符号 & 用法不同；没有函数调用所需的事件实参和调用括号。
    // ROS 内部绑定这两项，定时到期且回调队列被处理时，可理解为执行 this->execFSMCallback(event)。
    // event 由 ROS 提供给回调，不是这里手动传入；this 也不指向 nh 或 planner_manager_。
    // 未写出的选项使用默认 oneshot=false、autostart=true：重复定时并自动启动。
    // 实际函数体仍需由 main() 的 ros::spin() 等回调处理入口调度；createTimer() 不等于立刻执行一次状态机。
    exec_timer_ = nh.createTimer(ros::Duration(0.01), &SCANReplanFSM::execFSMCallback, this);
    // 定时任务：采样当前轨迹碰撞风险，周期 0.05 s（目标 20 Hz），发现风险时尝试重规划或急停。
    // 绑定方式相同：选择本类 checkCollisionCallback，并在同一个 this 状态机对象上运行，共享其轨迹和状态。
    // safety_timer_ 保存这项任务的句柄；两项定时器的目标周期不同，但本项目没有为它们各建一个独立执行线程。
    safety_timer_ = nh.createTimer(ros::Duration(0.05), &SCANReplanFSM::checkCollisionCallback, this);

    std::string body_pose_topic;
    // 参数读取：机身 Odometry 接口；默认仿真 /quad_0/body_pose，真机由父 launch 切到
    // /LIO/odom_vehicle。
    // 配置来源：run.launch → /body_pose_topic；正常默认启动取 /quad_0/body_pose。
    // 缺参回退：std::string("/quad_0/body_pose")；只有参数服务器中没有该键时才使用。
    ros::param::param<std::string>("/body_pose_topic", body_pose_topic, std::string("/quad_0/body_pose"));
    // 运行连接（接收入口）：机身 Odometry 接口，实际名称由 /body_pose_topic 指定；默认仿真
    // /quad_0/body_pose，接入规划/控制/感知/显示相应回调。
    // 队列容量：1 条消息，不是发布/规划频率。 到达后交给 odometryCallback。
    odom_sub_ = nh.subscribe(body_pose_topic, 1, &SCANReplanFSM::odometryCallback, this);
    // 运行连接（接收入口）：执行进度冻结反馈：闭环控制器发布、FSM 接收；true 表示原地转向时暂停参考
    // 推进。
    // 队列容量：10 条消息，不是发布/规划频率。 到达后交给 go2ExecutionFrozenCallback。
    go2_execution_frozen_sub_ = nh.subscribe("/planning/go2_execution_frozen", 10, &SCANReplanFSM::go2ExecutionFrozenCallback, this);

    // 运行连接（输出入口）：可执行局部轨迹接口：FSM 发布控制点、knots 和起始时刻，执行控制器接收后
    // 重建样条。
    // 队列容量：10 条消息，不是发布/规划频率。
    bspline_pub_ = nh.advertise<scan_planner::Bspline>("/planning/bspline", 10);
    // 运行连接（输出入口）：规划调试消息输出；当前主要刷新时间戳，a~e 未填入实际性能统计。
    // 队列容量：100 条消息，不是发布/规划频率。
    data_disp_pub_ = nh.advertise<scan_planner::DataDisp>("/planning/data_display", 100);
    // 运行连接（输出入口）：规划器私有机身几何显示接口，默认 /scan_planner_node/self_inflation，发
    // 布前后圆柱 Marker。
    // 队列容量：10 条消息，不是发布/规划频率。 锁存最后一条消息，供新连接的订阅者立即取得。
    self_inflation_pub_ = nh.advertise<visualization_msgs::Marker>("self_inflation", 10, true);

    // 目标入口按模式互斥选择：mode 1 监听点击；mode 2 直接规划预设关键点；mode 3 监听外部参考路径。
    if (navi_mode_ == NAVI_MODE::MANUAL_TARGET)
      // 运行连接（接收入口）：RViz 手动单目标入口（PoseStamped）；mode 1 的 FSM 据此建立导航任务。
      // 队列容量：1 条消息，不是发布/规划频率。 到达后交给 rvizGoalCallback。
      goal_sub_ = nh.subscribe("/move_base_simple/goal", 1, &SCANReplanFSM::rvizGoalCallback, this);
    else if (navi_mode_ == NAVI_MODE::PRESET_TARGET)
    {
      ros::Duration(1.0).sleep();
      // mode 2 需要有效起点才启动首段；等待期间主动处理回调以接收机身位姿，此时还未返回 main() 的 spin()。
      // ros::ok() 在正常运行时为 true，节点关闭（如 Ctrl+C 或 ros::shutdown()）后为 false，结束等待。
      while (ros::ok() && !have_odom_)
        // spinOnce() 处理当前可用的一轮回调后返回，不是仅执行一个回调；位姿回调会将 have_odom_ 置 true。
        ros::spinOnce();
      // 已有机身状态后启动关键点序列的首段参考规划，后续执行与切点由状态定时器处理。
      planGlobalTrajbyGivenWps();
    }
    else if (navi_mode_ == NAVI_MODE::REFERENCE_PATH)
      // 运行连接（接收入口）：外部路线参考入口（Path）；mode 3 从中组织局部规划所沿用的全局参考。
      // 队列容量：1 条消息，不是发布/规划频率。 到达后交给 pathCallback。
      path_sub_ = nh.subscribe("/initial_path", 1, &SCANReplanFSM::pathCallback, this);
    else
      cout << "Wrong navi_mode_ value! navi_mode_=" << navi_mode_ << endl;
  }

  /// @brief 将配置关键点保存为当前活动序列，先为首个关键点生成参考轨迹并触发规划。
  void SCANReplanFSM::planGlobalTrajbyGivenWps()
  {
    std::vector<Eigen::Vector3d> wps = preset_waypoints_;

    for (size_t i = 0; i < wps.size(); i++)
    {
      // 参数依次为：关键点 XYZ（世界系 m）、RGBA 颜色、显示球直径（0.3 m）、标记编号 i。
      // 当前实现的 RGB 固定为白色，传入颜色只取透明度 A；具体处理见 displayGoalPoint() 定义。
      visualization_->displayGoalPoint(wps[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
      ros::Duration(0.001).sleep();
    }

    active_waypoints_ = wps;
    current_wp_ = 0;
    trigger_ = true;
    init_pt_ = odom_pos_;

    if (planNextWaypoint())
    {
      changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory to first preset waypoint!");
    }
  }

  /// @brief 接收 RViz 单目标；首次机身位姿未到达前不接受目标。
  /// 将 PoseStamped 封装成 Path 后内部调用 waypointCallback()，没有额外发布一次话题。
  // geometry_msgs：ROS 标准几何消息包的命名空间；PoseStampedConstPtr：只读位姿消息指针的类型别名
  void SCANReplanFSM::rvizGoalCallback(const geometry_msgs::PoseStampedConstPtr &msg)
  {
    if (!msg)
      return;

    if (!rviz_height_ready_)
    {
      // ROS_WARN 输出警告级日志，每次执行到此处都可打印，没有时间限频；是否退出由后面的 return 决定。
      ROS_WARN("[SCANReplanFSM] Ignore RViz goal before receiving initial body pose.");
      return;
    }

    // 把 RViz 的单个 PoseStamped 目标包装成仅含一个点的 Path，以复用后面的目标处理函数。
    // Path 消息包含自身的 header，以及 poses 列表；列表的每个元素是一份完整 PoseStamped。
    // path 是本函数的局部共享智能指针；new 创建实际 Path 消息，并通过 path 的构造接管它。
    // 这不是调用名为 path 的函数；刚创建时 path->poses 是空列表，也尚未计算出规划路线。
    // C++ 直接初始化格式为“类型 变量名(构造实参);”：本句类型是 PathPtr，变量名是 path。
    // 括号内 new nav_msgs::Path 先构造消息对象，表达式结果是 nav_msgs::Path*，即新对象的地址。
    // PathPtr 支持接收这种原始指针的构造函数，因此用这个地址初始化智能指针，并接管对象所有权。
    // 可分两步理解：nav_msgs::Path* p = new nav_msgs::Path; 然后 nav_msgs::PathPtr path(p);。
    // “接管地址并管理对象”来自智能指针构造函数的行为，不是所有带括号的初始化都会执行这个动作。
    nav_msgs::PathPtr path(new nav_msgs::Path);

    // 复制输入目标的消息头到 Path 的总消息头，保留时间戳、参考坐标系等信息；此赋值不做坐标变换。
    path->header = msg->header;

    // -> 访问 Path 对象的 poses 列表；push_back() 是向列表末尾追加一个元素，空列表因此变为 1 个点。
    // *msg 取得指针指向的整份 PoseStamped 消息，复制进列表；追加的是消息内容，不是智能指针本身。
    // 例如目标为 (1,2,0.5)，现在 path->poses[0].pose.position 就保存这组 XYZ，原输入消息不被修改。
    path->poses.push_back(*msg);

    // 在当前状态机上直接调用目标处理函数，立即继续生成参考并更新任务标志；这里没有再发布一个话题。
    // path 可转换为该函数要求的只读 Path 指针，仍指向同一份 Path；被调函数的形参名也叫 msg，
    // 但那是另一函数里的 Path 消息指针，与本函数收到的 PoseStamped 指针不是同一个形参。
    waypointCallback(path);
  }

  /// @brief 使用输入首点的 XY 和首次机身位姿的 z 设置目标，生成多项式参考。
  /// 成功后检查目标占用、设置 have_target_ 并选择生成新轨迹或重规划状态。
  // mode 1 由 rvizGoalCallback() 内部传入单点 Path；这里的 msg 指向 Path，目标位姿在 msg->poses[0] 中。
  void SCANReplanFSM::waypointCallback(const nav_msgs::PathConstPtr &msg)
  {
    if (!msg || msg->poses.empty())
    {
      // 与 ROS_WARN 相比，THROTTLE 按首参指定的 ROS 时间间隔限频：1.0 表示此处连续触发时约每秒最多一条。
      // 只限制日志，不等待 1 s、不降低回调频率；下面的 return 每次仍会执行，各处日志独立限频。
      ROS_WARN_THROTTLE(1.0, "[waypointCallback] Empty waypoint message, ignore.");
      return;
    }

    if (msg->poses[0].pose.position.z < -0.1)
      return;

    cout << "Triggered!" << endl;
    trigger_ = true;
    init_pt_ = odom_pos_;

    bool success = false;
    end_pt_ << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, rviz_goal_height_;
    // planGlobalTraj()：按首尾状态生成从当前位置到目标的全局参考轨迹，保存到管理器，返回是否生成成功。
    // 此处先建立引导参考，局部避障轨迹由后续重规划流程生成；六个实参依次给出首尾的位置、速度、加速度边界。
    // Eigen::Vector3d::Zero() 是三维零向量 (0,0,0)，不是消息队列；三个零分别用于下方三个导数边界。
    // 本入口不提供测量加速度，因此起始加速度采用零近似；终点速度/加速度设零以形成停下来的参考端点。
    // 这些零只约束首尾对应导数，不要求整段轨迹的速度或加速度都为零。
    success = planner_manager_->planGlobalTraj(
        odom_pos_,                  // 起点位置：当前世界系 XYZ，单位 m。
        odom_vel_,                  // 起点速度：当前观测线速度，单位 m/s，可非零。
        Eigen::Vector3d::Zero(),     // 起点加速度：假设 (0,0,0)，单位 m/s²。
        end_pt_,                    // 终点位置：本次导航目标 XYZ，单位 m。
        Eigen::Vector3d::Zero(),     // 终点速度：期望 (0,0,0)，单位 m/s。
        Eigen::Vector3d::Zero());    // 终点加速度：期望 (0,0,0)，单位 m/s²。

    // 参考生成成功后，检查其终点是否被占用；若已占用，沿参考向后找可用点并调整目标/有效时长。
    // adjustGlobalTargetIfOccupied() 找到替代点或无需调整时返回 true，已占用且找不到可用点时返回 false。
    if (success)
      success = adjustGlobalTargetIfOccupied();

    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

    if (success)
    {

      /*** display ***/
      // 到这里，全局多项式参考已经生成并通过上述终点处理；下面先准备显示数据，再安排局部规划。
      // 此处 success 不表示面向新目标的局部 B 样条已生成或已发布给控制器。
      // 将连续的全局参考按时间取样为 XYZ 点列表，供 RViz 绘制；本循环不等待，也不驱动机器人运动。
      // constexpr 表示编译期可确定且不可修改的常量；此处固定轨迹时间轴上的采样间隔为 0.1 s。
      constexpr double step_size_t = 0.1;
      // 参考有效时长（s）除以采样间隔并向下取整，得到本次显示采样点数。
      // floor() 朝负无穷方向取整，例如 floor(3.8)=3、floor(-3.2)=-4；此处结果再存入整数 i_end。
      int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
      // 创建已有 i_end 个三维点元素的列表，随后用下标逐点填入采样结果；沿用原变量拼写 gloabl_traj。
      vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        // evaluate(t) 返回参考在相对时间 t 的世界系 XYZ（m）；依次取 t=0、0.1、0.2…，不专门补采终点。
        // 此函数按已生成的多项式参考计算 t 秒处的位置，返回 Eigen::Vector3d，保存到当前显示采样点。
        gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
      }

      // setZero() 将已有 end_vel_ 的三个速度分量置为 (0,0,0)，表达期望在目标处停止，单位 m/s。
      // 当前版本此缓存只写未读；这句不会重新计算上面已生成的参考，也不会立即改变实际机身速度。
      // 实际参考终速由前面 planGlobalTraj() 的第 5 个零向量实参约束；局部终速另由 getLocalTarget() 设置。
      end_vel_.setZero();
      // 全局任务就绪：已有目标和全局参考，允许状态机继续局部规划；不代表已得到新目标的可执行局部轨迹。
      have_target_ = true;
      // 全局目标/参考刚更新：通知下一次局部 B 样条优化重建候选初值，不是说上面的全局参考尚未生成。
      // 不是“允许重规划”开关；callReboundReplan() 尝试后即清除它，同一目标仍可继续重规划。
      have_new_target_ = true;

      /*** FSM ***/
      // changeFSMExecState() 修改状态并打印迁移日志；"TRIG" 表示此次请求由新目标/路线的接入处理发起，不会立即执行新状态的算法。
      // 两处 "TRIG" 含义相同：目标/参考已准备好，接入代码请求开始局部规划或更新已有局部轨迹；标签只用于日志。
      // 真正决定改到哪个状态的是第一个实参 GEN_NEW_TRAJ / REPLAN_TRAJ，而不是第二个字符串。
      // 等待任务时收到目标，准备生成新轨迹；已有轨迹在执行时收到新目标，准备重规划。
      // 此处安排的是面向新目标的局部轨迹生成/重规划；已有全局参考负责引导后续局部目标选择。
      if (exec_state_ == WAIT_TARGET)
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      else if (exec_state_ == EXEC_TRAJ)
        changeFSMExecState(REPLAN_TRAJ, "TRIG");
      // 若当前仍为 INIT，此处保持该状态，后续执行回调根据位姿/目标标志继续推进。

      // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      // 显示采样后的参考路径；参数为 XYZ 点列表、Marker 尺寸尺度（0.1 m）、标记编号 0。
      // 此处 0.1 是显示尺寸，区别于上方 step_size_t 的 0.1 s；不改变参考轨迹本身。
      // 这是 Marker 显示输出；控制器执行的新局部轨迹要等 callReboundReplan() 成功后通过 /planning/bspline 发布。
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
      // mode 1 此失败分支仅打印日志；不会自动切急停，也未回滚先前已写入的 end_pt_/global_data_，不要视作完整安全恢复。
      ROS_ERROR("Unable to generate global trajectory!");
    }
  }

  /// @brief 按给定路径点构造多段全局参考，更新目标标志和可视化；返回参考构造是否成功。
  bool SCANReplanFSM::planGlobalTrajByWaypoints(const std::vector<Eigen::Vector3d> &waypoints)
  {
    if (waypoints.size() < 2)
    {
      ROS_WARN("[planGlobalTrajByWaypoints] Reference path requires at least two points.");
      return false;
    }

    // back() 是 vector 的成员函数，访问最后一个元素，不会删除它；调用前必须保证列表非空。
    // 此处前面已要求至少两个点，因此将最后一个世界系 XYZ 点作为本段参考的最终目标，复制给 end_pt_。
    end_pt_ = waypoints.back();
    // vector 的区间构造：复制 [begin()+1, end()) 内的元素，创建新的 reference_waypoints 列表。
    // begin() 指向首元素，+1 从第二个点开始；end() 指向末元素之后，区间不包含这个尾后位置。
    // 例如原列表为 [P0,P1,P2]，新列表为 [P1,P2]，原列表不变；P0 留给下方 waypoints.front() 单独作起点。
    // 此处 waypoints 由 mode 3 的 pathCallback() 处理外部 /initial_path 后传入，reference_waypoints 是本地复制列表。
    std::vector<Eigen::Vector3d> reference_waypoints(waypoints.begin() + 1, waypoints.end());

    // 按不同编号 i 依次发布各点 Marker；已显示的点默认不自动过期，后续点到达后可累计同时显示。
    for (size_t i = 0; i < waypoints.size(); i++)
    {
      visualization_->displayGoalPoint(waypoints[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
      // 当前线程等待 0.001 s（1 ms）后再发下一点，不是让本点只显示 1 ms；RViz 渲染时刻独立于此循环。
      ros::Duration(0.001).sleep();
    }

    // planGlobalTrajWaypoints()：从单独给出的起点和后续途经点列表构造多段全局多项式参考，存入 global_data_。
    // 返回 true/false 表示参考生成是否成功；局部避障与可执行 B 样条仍由后续状态机规划。
    bool success = planner_manager_->planGlobalTrajWaypoints(
        waypoints.front(),          // front() 访问原列表第一个点 P0，不删除它；作为参考起点（世界系 m）。
        Eigen::Vector3d::Zero(),     // 起点速度 (0,0,0)，单位 m/s。
        Eigen::Vector3d::Zero(),     // 起点加速度 (0,0,0)，单位 m/s²。
        reference_waypoints,        // 上方构造的 [P1,P2,…]，含最终目标；不重复传入已单列的起点 P0。
        Eigen::Vector3d::Zero(),     // 终点速度 (0,0,0)，单位 m/s。
        Eigen::Vector3d::Zero());    // 终点加速度 (0,0,0)，单位 m/s²。

    // 管理器未能生成参考时，结束本次参考处理并向调用方返回失败；打印 ERROR 本身不会关闭节点。
    if (!success)
    {
      ROS_ERROR("Unable to generate global trajectory from waypoints!");
      return false;
    }

    // 与 mode 1 相同的终点处理：已占用时沿参考向后找可用点，成功则更新 end_pt_ 和参考有效时长。
    // 先执行函数再用 ! 取反：返回 false → 条件为 true → 本函数返回 false；返回 true → 继续下面的采样显示。
    if (!adjustGlobalTargetIfOccupied())
      return false;

    // mode 3 上一步已将外部路径点重构为内部全局多项式参考；此处采样这条新参考，仅用于 RViz 显示。
    // 0.1 s 不是为对齐外部轨迹采样间隔；当前入口没有使用外部各点时间戳建立时间约束。
    constexpr double step_size_t = 0.1;
    int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
    // gloabl_traj 是本地显示点列表；下方只填它，不修改管理器中保存的连续 global_traj_ 参考。
    std::vector<Eigen::Vector3d> gloabl_traj(i_end);
    for (int i = 0; i < i_end; i++)
    {
      gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
    }

    end_vel_.setZero();
    have_target_ = true;
    have_new_target_ = true;
    // 与 mode 1 一样，参考就绪后需要显示并通知局部规划；这里发布显示 Marker，不是执行轨迹命令。
    visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, static_cast<int>(waypoints.size()) - 1);

    return true;
  }

  /// @brief mode 2 为 current_wp_ 指定的关键点准备本段全局参考，并更新任务标志与显示。
  /// 首次由 planGlobalTrajbyGivenWps() 调用；后续由 EXEC_TRAJ 切点分支先增加索引再调用。
  /// 函数名虽有 Next，但这里不会递增 current_wp_、判断是否到达或直接发布可执行局部 B 样条。
  bool SCANReplanFSM::planNextWaypoint()
  {
    // 1. 检查当前目标索引；active_waypoints_ 是从预设 YAML 点复制出的本次任务序列。
    if (current_wp_ < 0 || current_wp_ >= (int)active_waypoints_.size())
    {
      ROS_WARN("[navi_mode=%d] No active waypoint to plan.", navi_mode_);
      return false;
    }

    // 2. 只选序列中的当前一个点作终点，不在此一次规划完整点序列；占用检查之后还可能回退此终点。
    end_pt_ = active_waypoints_[current_wp_];
    // 组织本段参考起始边界，使用当前机身位置及可用的观测/旧轨迹导数。
    setStartStateFromOdomOrCurrentTraj();

    // 3. 为当前起点到这个关键点生成多项式参考，期望终点速度/加速度为零；这一步还未完成局部避障。
    bool success = planner_manager_->planGlobalTraj(
        start_pt_,
        start_vel_,
        start_acc_,
        end_pt_,
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero());

    if (!success)
    {
      ROS_ERROR("[navi_mode=%d] Unable to generate trajectory to waypoint %d.", navi_mode_, current_wp_ + 1);
      return false;
    }

    // 4. 当前参考终点已占用时沿参考回退到可用点，无法处理则返回失败。
    if (!adjustGlobalTargetIfOccupied())
      return false;

    // 5. 每隔参考时间 0.1 s 求一个 XYZ，仅准备可视化采样列表，不在循环中等待或驱动机身运动。
    constexpr double step_size_t = 0.1;
    int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
    std::vector<Eigen::Vector3d> gloabl_traj(i_end);
    for (int i = 0; i < i_end; i++)
    {
      gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
    }

    // 6. 全局参考就绪，更新任务标志；局部轨迹仍需调用方切到 GEN_NEW_TRAJ 后由执行回调生成。
    // end_vel_ 是只写未读的保留终速缓存；实际参考终速已由上面的零向量实参约束。
    end_vel_.setZero();
    have_target_ = true;
    have_new_target_ = true;
    visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, current_wp_);
    ROS_INFO("[navi_mode=%d] Planning to waypoint %d/%zu: [%.2f, %.2f, %.2f].",
             navi_mode_, current_wp_ + 1, active_waypoints_.size(), end_pt_(0), end_pt_(1), end_pt_(2));

    // true 表示当前关键点的全局参考准备成功，不表示已到达该点或局部 B 样条已发布。
    return true;
  }

  /// @brief 仅 mode 2 按活动关键点逐个推进；mode 3 以输入路径构造参考而不走同一切点逻辑。
  bool SCANReplanFSM::isWaypointSequenceMode() const
  {
    return navi_mode_ == NAVI_MODE::PRESET_TARGET;
  }

  /// @brief 对已占用的参考终点沿参考轨迹向后搜索空闲点，并同步缩短参考有效时间。
  /// 返回 true 也可能表示无需调整；本函数对越界返回值的处理应与局部目标查询区分。
  bool SCANReplanFSM::adjustGlobalTargetIfOccupied()
  {
    // 取得当前局部占用地图；这是共享地图入口，不是复制整张地图。
    auto map = planner_manager_->grid_map_;
    // 引用管理器里的参考数据，下面改有效时长/进度缓存会直接作用于这份参考。
    auto &global_data = planner_manager_->global_data_;
    // T：本次检查开始时的参考有效总时长（s），是轨迹相对时间上限，不是当前 ROS 时刻。
    const double duration = global_data.global_duration_;
    // 没有地图或参考短于 0.001 s 时跳过本次处理；true 只表示不在这里阻止后续，不是安全证明。
    if (!map || duration < 1e-3)
      return true;

    // 用参考时间均匀取样；目标最大间隔 0.05 s，不是空间后退 0.05 m，也不会在循环里等待。
    constexpr double sample_dt = 0.05;
    // N 是区间数：向上取整且至少为 1，实际间隔 Δt=T/N<=0.05 s；循环至多检查 N+1 个候选点（含两端）。
    // ceil() 向正无穷方向取整，如 ceil(3.2)=4、ceil(-3.2)=-3；这里输入 double，返回的整数值仍是 double 类型。
    // static_cast<int>(...) 再把结果显式转为 int；例 1.03/0.05=20.6 → ceil 得 21.0 → 转为 21 个区间。
    // static_cast<目标类型>(表达式) 是 C++ 显式类型转换，不是普通函数；如 static_cast<int>(3.8) 得 3，不做四舍五入。
    // 本句先用 ceil() 确定向上取整的数值，再转换类型；转为 int 后也能与外层 std::max() 的整数 1 匹配。
    const int sample_num = std::max(1, static_cast<int>(std::ceil(duration / sample_dt)));
    // P(T) 是当前参考终点；P(T-Δt) 是轨迹上稍早的位置，用二者方向估计到达终点的 yaw。
    const Eigen::Vector3d final_pt = global_data.global_traj_.evaluate(duration);
    const Eigen::Vector3d final_prev = global_data.global_traj_.evaluate(duration * (sample_num - 1) / sample_num);
    // 按该 yaw 查询双中心膨胀占用：0=未被膨胀障碍占用，1=占用，-1=查询超出当前地图范围。
    // 将 final_pt 作为机身规划参考位置，末段方向作为 yaw；圆柱半径/上下距离已编码到障碍膨胀层。
    // 此处只查前后两个圆柱中心是否落入该层，近似判断“双圆柱与原始障碍相交”，不再重复拿完整圆柱叠加膨胀区。
    const int final_occ = map->getInflateOccupancy(final_pt, estimateYawFromSegment(final_prev, final_pt));
    // 保留现有判据：0 不调整，-1（越界）也直接返回 true；只有正占用值才进入下方回查。
    // 因此此处 true 不能一律理解为“终点已在地图内确认空闲”。
    if (final_occ <= 0)
      return true;

    // 从终点向轨迹起点枚举：i=N,N-1,...,0，对应 t=T,T-Δt,...,0；不是让机器人实际倒车。
    // 例 T=2 s、N=40：依次查 P(2.00)、P(1.95)、P(1.90)...，遇到首个可用候选即结束。
    // 若前两个占用、P(1.90) 可用，则目标改为 P(1.90)，参考有效时长缩到 1.90 s，立即返回 true。
    // 后退一个候选仍碰撞时继续检查更早候选；这里只验证候选终态占用，抵达它的避障可达性仍由后续局部规划判断。
    for (int i = sample_num; i >= 0; --i)
    {
      // 当前候选时刻；i 递减让 t 减小，回到原参考上更早的位置。
      const double t = duration * i / sample_num;
      // 候选的前一参考时刻，i=0 时夹到 0；它不是上一次循环刚查过的更晚时刻。
      const double prev_t = duration * std::max(0, i - 1) / sample_num;
      // pt 是候选目标位置（世界系 m），prev_pt 仅帮助估计原参考的前进朝向。
      const Eigen::Vector3d pt = global_data.global_traj_.evaluate(t);
      const Eigen::Vector3d prev_pt = global_data.global_traj_.evaluate(prev_t);

      // 搜索虽然按时间倒序，但朝向仍按 prev_pt→pt 的原前进方向估计；XY 方向退化时回退当前机身 yaw。
      // 只有 ==0 接受候选；占用或越界都继续回查，且只保证这个采样姿态通过查询，不保证整段参考无障碍。
      if (map->getInflateOccupancy(pt, estimateYawFromSegment(prev_pt, pt)) == 0)
      {
        // 旧目标只用于下面的日志对照，不用于回滚或重新规划。
        const Eigen::Vector3d raw_end = end_pt_;
        // 将任务目标改为当前可用候选，所选的是倒序采样遇到的第一个点，不是全空间欧氏最近点。
        end_pt_ = pt;
        // 将参考的有效范围截到 [0,t]；多项式系数未重算，原曲线 t 之后的部分不再作为有效参考使用。
        global_data.global_duration_ = t;
        // 裁剪参考进度缓存，确保不保留超出新有效末时刻的进度；原值较小时保持原值。
        global_data.last_progress_time_ = std::min(global_data.last_progress_time_, t);
        ROS_WARN("[global target] Target [%.2f, %.2f, %.2f] is occupied; use backward collision-free point [%.2f, %.2f, %.2f].",
                 raw_end(0), raw_end(1), raw_end(2), end_pt_(0), end_pt_(1), end_pt_(2));
        return true;
      }
    }

    // 所有离散候选均未通过 ==0 检查，返回失败给调用方；这不等同于证明连续曲线上绝无空闲位置。
    // 此处失败仅表示这条参考的离散候选都未通过；本函数不做目标邻域多方向搜索，也不自动停车或证明全地图不可达。
    ROS_ERROR("[global target] Target is occupied, and no collision-free point was found along the global trajectory.");
    return false;
  }

  /// @brief mode 3 的 Path 输入：剔除过近的中间点，给 z 添加 body_height_ 后构造参考。
  /// 需要已有 odom；输入路径点应采用规划器使用的世界坐标系。
  void SCANReplanFSM::pathCallback(const nav_msgs::PathConstPtr &msg)
  {
    if (!msg || msg->poses.empty())
    {
      // 对这处重复空路径警告限频，间隔 1.0 s；不会等待，随后直接结束本次处理。
      ROS_WARN_THROTTLE(1.0, "[pathCallback] Received empty /initial_path, ignore.");
      return;
    }

    if (!have_odom_)
    {
      // 尚无机身状态时最多约每 1.0 s 输出一次本警告，避免反复收到路径造成刷屏。
      ROS_WARN_THROTTLE(1.0, "[pathCallback] No odometry yet, cannot plan global trajectory.");
      return;
    }

    trigger_ = true;
    // mode 3 的 body_height_ 是沿世界 Z 的固定参考点偏移（默认 0.4 m），不是补偿里程计测不到高度。
    // 它适用于把地面/通行面路线抬到机身规划参考点的接口约定；本代码不会判断上游路径属于哪种参考点。
    // 若上游已给机身参考点高度，应核对偏移以免重复抬高；加常量保留输入高差，不会把路径压成水平面。
    // 先保存输入路线的末点作为目标，避免后面的距离筛选将终点丢弃；此时尚未做占用检查或目标调整。
    end_pt_ << msg->poses.back().pose.position.x,
        msg->poses.back().pose.position.y,
        msg->poses.back().pose.position.z + body_height_;

    std::vector<Eigen::Vector3d> waypoints;
    // reserve(N) 是 vector 的容量预留函数，提前准备至少容纳 N 个点的空间，减少后续 push_back() 的反复扩容。
    // 此处 N 为外部 Path 的位姿数量；reserve() 不创建列表元素，waypoints.size() 此时仍为 0，筛点通过后才逐个追加。
    // 区别于前面的 resize(N)：resize 会把元素个数改为 N，因此可用下标赋值；reserve 后不能直接访问尚不存在的 [i]。
    waypoints.reserve(msg->poses.size());
    // 输入筛点按空间距离（m）进行，首点保留、末点随后补齐；与显示时的 0.1 s 时间采样不同。
    // min_dist：后续候选点距“上一个已保留点”至少 0.5 m 才保留；使用三维欧氏距离，不要求严格等间距。
    // 这里用固定阈值控制输入点密度；后续这些点用于分段多项式构造，源码未给出为何必须取 0.5 m 的推导。
    // 阈值更小通常保留更多细节并增大参考构造规模；更大则点更少，但可能略过局部弯折，不保证保留全部拐点。
    // 它是路线筛点间距，不是障碍安全距离；当前也没有暴露为 launch 参数。
    constexpr double min_dist = 0.5;
    // last_wp：缓存上一个已加入 waypoints 的点（已加高度偏移），不是上一个遍历到但可能被丢弃的输入点。
    // 此处尚未赋有效坐标，首个点被保留时才赋值；后续距离比较都以它为基准。
    Eigen::Vector3d last_wp;
    // first：是否还没保留首点。true 时无条件保留首点，随后置 false；也通过 || 短路避免首次读取 last_wp。
    bool first = true;

    // 本入口只取外部 Path 的位置并加机身高度偏移，未用各点 header.stamp 或姿态重建外部执行时序。
    // 范围 for：遍历 msg 指向的 Path 消息的 poses 列表，每轮用 pose_stamped 访问其中一个位姿消息。
    // auto 让编译器推导元素类型，这里是 geometry_msgs::PoseStamped；不是运行时随意改变类型。
    // const auto & 表示只读引用：& 避免逐点复制消息，const 禁止经此引用修改原列表元素。
    // 冒号 : 表示“依次遍历右侧范围”，不是赋值；可理解为每轮 const PoseStamped& pose_stamped = 当前元素。
    for (const auto &pose_stamped : msg->poses)
    {
      Eigen::Vector3d wp;
      wp(0) = pose_stamped.pose.position.x;
      wp(1) = pose_stamped.pose.position.y;
      // 对每个原始路线点同样加一次高度偏移；不是在已抬高的 end_pt_ 上再加一次。
      wp(2) = pose_stamped.pose.position.z + body_height_;

      // wp-last_wp 得到三维差向量，Eigen 的 norm() 求欧氏长度 sqrt(dx²+dy²+dz²)，因此这里是点距（m）。
      // 与前面的 squaredNorm() 不同，norm() 含开方；首点时 first=true，|| 短路跳过尚无有效值的 last_wp。
      if (first || (wp - last_wp).norm() >= min_dist)
      {
        waypoints.push_back(wp);
        last_wp = wp;
        first = false;
      }
    }

    // 循环也会筛选输入末点：若它距上一个保留点不足 0.5 m，可能未被加入，但仍需保留路线终点。
    // back() 是筛选后列表的最后一个点；若它与 end_pt_ 的距离大于 1e-6 m，就把终点补到列表末尾。
    // 1e-6 是判断两点近似相同的容差，不是另一个筛点间距；终点已保留时不重复追加。
    // 因此最后一段可短于 0.5 m；例如同一直线输入 0、0.6、0.8 m，先筛为 0、0.6，再补回终点 0.8。
    if ((waypoints.back() - end_pt_).norm() > 1e-6)
      waypoints.push_back(end_pt_);

    bool success = planGlobalTrajByWaypoints(waypoints);

    if (success)
    {
      /*** FSM ***/
      if (exec_state_ == WAIT_TARGET)
      {
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      }
      else if (exec_state_ == EXEC_TRAJ)
      {
        changeFSMExecState(REPLAN_TRAJ, "TRIG");
      }

      ROS_INFO("==========================================\n");
    }
    else
    {
      ROS_ERROR("❌ Unable to generate global trajectory!");
    }
  }

  /// @brief 保存实际机身位置、速度和姿态，并在 mode 1 首次接收时锁定 RViz 目标高度。
  /// 同时发布用于显示的碰撞几何 Marker。
  void SCANReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    // 机身状态输入本身提供 z；这里直接采用它，不添加 body_height_，实际高度精度由上游定位/仿真决定。
    odom_pos_(2) = msg->pose.pose.position.z;

    if (navi_mode_ == NAVI_MODE::MANUAL_TARGET && !rviz_height_ready_)
    {
      rviz_goal_height_ = odom_pos_(2);
      rviz_height_ready_ = true;
      ROS_INFO("[SCANReplanFSM] Set RViz goal height from initial body_pose z: %.3f", rviz_goal_height_);
    }

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    //odom_acc_ = estimateAcc( msg );

    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    have_odom_ = true;
    publishSelfInflationMarker();
  }

  /// @brief 接收闭环控制器是否正在原地转向的标志，供规划端轨迹时钟补偿使用。
  void SCANReplanFSM::go2ExecutionFrozenCallback(const std_msgs::BoolConstPtr &msg)
  {
    // 保存闭环控制器发来的 Bool：true 表示要求暂停轨迹进度，false 表示解除；不是本函数计算出的时长。
    go2_execution_frozen_ = msg->data;
  }

  /// @brief 冻结局部轨迹的相对进度 t_cur = now - start_time_：将起始时间戳推迟 dt，抵消 now 的增加。
  /// 每次调用都刷新上次时刻，只有冻结标志为 true 时才补偿轨迹时间；不停止 ROS 时钟或修改控制点。
  /// 状态机和碰撞检查定时回调都会调用本函数，未冻结时也照常调用，不是一次冻结对应一次调用。
  void SCANReplanFSM::updateLocalTrajTimeFreeze()
  {
    // ROS 当前时刻仍正常推进；要冻结的是规划轨迹的执行进度。
    const ros::Time now = ros::Time::now();
    // 相邻两次函数调用的间隔（s），通常是较小正数，不是从开始冻结累计到现在的时长。
    double dt = (now - last_freeze_update_time_).toSec();
    // 保存的是“本次函数调用时刻”，不是冻结结束时刻；下一次调用时，它就成了“上次调用时刻”。
    // 未冻结时也刷新，避免下次进入冻结时把此前正常运行的整段时间误计入 dt。
    // 本行只更新计时间隔的基准，不改变冻结标志；实际是否补偿由后面的 go2_execution_frozen_ 判断。
    last_freeze_update_time_ = now;

    // 防御性检查：ROS 时间未推进/回跳时 dt<=0；回调长时间未运行或时钟前跳时可能 dt>0.2。
    // 本实现跳过这一次补偿；0.2 s 不限制总冻结时长，长冻结可由许多有效小步累计补偿。
    if (dt <= 0.0 || dt > 0.2)
      return;

    // LocalTrajData 保存当前局部 B 样条及其编号、起始时间、时长等信息，定义见 plan_container.hpp。
    // info 是指向管理器现有 local_data_ 的普通指针；& 取地址，不新建或复制轨迹，函数退出后原数据仍保留。
    // info->成员 就是在访问原 local_data_ 的成员，因此下面修改 start_time_ 会直接更新管理器保存的时间。
    LocalTrajData *info = &planner_manager_->local_data_;
    // start_time_ 是可补偿的绝对时间基准，区别于用于轨迹求值的相对时间 t_cur；此处不移动空间起点。
    // 例如 now=12 s、start_time_=10 s 时进度为 2 s；冻结一小步 dt=0.01 s 后，两者变为 12.01、10.01 s。
    // 差值仍为 2 s；若只保持 start_time_=10 s，进度反而会增加到 2.01 s，不能实现冻结。
    // 已有有效轨迹且控制器要求冻结时才做补偿；解除冻结后起始时间戳不再随本函数增加，进度继续推进。
    // 这是按调用间隔逐步补偿：用当前冻结标志决定是否计入整段 dt，未按标志切换的精确时刻拆分边界间隔。
    // 两项须同时成立：控制器要求冻结，且已有非近零的轨迹起始时间戳；toSec() 转成秒，1e-5 s 为 0.00001 s。
    // start_time_ 默认是零；保存局部轨迹时，管理器的 updateTrajInfo() 已写入当前 ROS 时刻，之后还可跨调用累计补偿。
    // 此处检查保存的时间戳是否非近零，不是检查本次是否修改过它，也不是判断轨迹已执行多久或完整验证轨迹有效性。
    if (go2_execution_frozen_ && info->start_time_.toSec() > 1e-5)
      info->start_time_ += ros::Duration(dt);
  }

  /// @brief 从当前机身四元数提取水平朝向，供显示或局部状态估计使用，单位 rad。
  double SCANReplanFSM::getOdomYaw() const
  {
    // odom_orient_ 来自里程计的机身姿态；对有效单位四元数，toRotationMatrix() 得到 3×3 旋转矩阵 R。
    // 按机身 +X 为前方、姿态将机身坐标旋转到世界系的约定，R 的各列是机身各坐标轴在世界系的方向。
    // col(0) 取第 0 列（第一列）：R*(1,0,0)，即机身前方在世界系的单位方向，不是位置或欧拉角。
    Eigen::Vector3d heading = odom_orient_.toRotationMatrix().col(0);
    // head<2>() 取向量前两个分量 (hx,hy)，模板参数 <2> 指长度；即前方方向在世界 XY 平面的投影。
    // squaredNorm() 求 hx²+hy²（不含 hz，也不开方）；此处是无量纲方向投影，不是以米计的点距。
    // 若投影长度小于 1e-4，水平方向不可靠（有效姿态下机身 +X 接近竖直），回退 yaw=0 rad。
    // 这个 0 是本函数选定的回退值，不代表实际机身一定朝世界 +X；也不是水平行走时 hz≈0 就会触发。
    if (heading.head<2>().squaredNorm() < 1e-8)
      return 0.0;
    // heading(0/1/2) 分别为 hx/hy/hz；atan2 的两个实参按 (y,x) 排列，故先传 heading(1) 再传 heading(0)。
    // 返回水平朝向角 yaw（rad，相对世界 +X，向 +Y 为正）；例如 heading=(0,1,0) 时为 π/2。
    return std::atan2(heading(1), heading(0));
  }

  /// @brief 由相邻轨迹点的 XY 方向估计 yaw，供方向相关的双中心碰撞查询使用。
  double SCANReplanFSM::estimateYawFromSegment(const Eigen::Vector3d &from, const Eigen::Vector3d &to) const
  {
    // 由 from 指向 to 的水平差向量 (dx,dy) 直接计算朝向，不是统计预测；此处仅考虑 yaw，不用两点的 z。
    // 称为“估计”是因为用这一小段参考的前进方向近似该位置的机身朝向，未直接使用真实执行姿态。
    Eigen::Vector2d diff(to(0) - from(0), to(1) - from(1));
    // 水平两点过近时方向不可靠：dx²+dy²<1e-8，即水平距离小于约 0.0001 m，回退当前里程计 yaw。
    if (diff.squaredNorm() < 1e-8)
      return getOdomYaw();
    // atan2(dy,dx) 返回相对世界 +X 轴的有符号角度（rad），能区分四个象限，也能处理 dx=0。
    return std::atan2(diff(1), diff(0));
  }

  /// @brief 按机身位置、yaw 和双圆柱配置生成显示 Marker；显示几何不代替地图碰撞判断。
  void SCANReplanFSM::publishSelfInflationMarker()
  {
    // std::max(a,b) 选较大值，不是相加；这里给显示尺寸设下限，防止配置负值产生负半径/负伸展长度。
    // 参数为非负数时原样使用，为负数时取 0；例如 max(0.0,0.25)=0.25，max(0.0,-0.1)=0.0。
    // 这些是局部显示用量，不会改写原配置成员，也不代替地图碰撞参数的校验。
    const double radius = std::max(0.0, self_double_cylinder_radius_);
    // 此处的参考位置就是 odom_pos_：机身状态 Odometry.pose.pose.position 报告的定位点，不由本函数推断物理中心。
    // z_up/z_down 是显示包络沿世界 Z 轴、相对该点向上/向下延伸的非负长度（m），不是离地高度或抬升指令。
    // 正常非退化尺寸下，圆柱顶面 z=odom_pos_(2)+z_up，底面 z=odom_pos_(2)-z_down；两个圆柱共用该高度范围。
    // 例如定位点 z=0.4 m，上下各 0.1 m，则显示包络覆盖 z∈[0.3,0.5] m；向下长度仍填正的 0.1。
    // 默认显示模型总厚度为 0.2 m，来自 launch 手工配置；这里没有读取 URDF 或按真实机身尺寸自动计算。
    // 模型厚度不同于机身离地高度 body_height_；是否覆盖躯干、腿或附加设备需按实际任务确定，不能由默认值推断。
    // 同名参数在 GridMap 中用于障碍膨胀：换算为机器人包络时上下方向需反射；当前上下相等，非对称时须另核对显示一致性。
    const double z_up = std::max(0.0, self_inflation_z_up_);
    const double z_down = std::max(0.0, self_inflation_z_down_);
    // height 是本函数算出的显示圆柱总高度，不是另外读取的实际机身高度，也不是成员 body_height_。
    // 它由上述 z_up+z_down 得到（默认 0.1+0.1=0.2 m），最低取 1e-3 m（1 mm），避免零高度的退化几何。
    const double height = std::max(1e-3, z_up + z_down);

    visualization_msgs::Marker marker;
    marker.header.frame_id = self_inflation_frame_id_.empty() ? "world" : self_inflation_frame_id_;
    marker.header.stamp = ros::Time::now();
    marker.ns = "self_inflation";
    marker.type = visualization_msgs::Marker::CYLINDER;
    // ADD 是 Marker 消息定义的动作常量：新 ns/id 则添加，已有相同 ns/id 则更新；此处不创建仿真碰撞物体。
    marker.action = visualization_msgs::Marker::ADD;
    // 消息默认 x/y/z=0，设 w=1 得到单位四元数：圆柱不旋转，轴线沿 marker.header.frame_id 的 +Z。
    marker.pose.orientation.w = 1.0;
    // CYLINDER 的 scale.x/y 是横截面沿 X/Y 的完整尺寸（m），圆形截面两者都填直径 2*radius，不是填半径。
    marker.scale.x = 2.0 * radius;
    marker.scale.y = 2.0 * radius;
    // scale.z 使用上面算出的 height=max(0.001,z_up+z_down)，所以与上下长度来自同一组配置，没有另取实际机身高度。
    // 例如半径 0.25、上下各 0.1，对应 scale=(0.5,0.5,0.2) m。
    marker.scale.z = height;
    // RGBA 各分量取 0~1；这里为半透明蓝色，alpha=0.4（0 为透明，1 为不透明）。
    marker.color.r = 0.1;
    marker.color.g = 0.6;
    marker.color.b = 1.0;
    marker.color.a = 0.4;
    // RViz 中本标记的有效期为 0.2 s；收到相同 ns/id 的更新会重新计时，持续发布可保持显示，停止更新后自动消失。
    // 不是本函数等待 0.2 s，也不决定轨迹/模型的运行时长；后面 id=0/1 的两个圆柱分别维护自己的有效期。
    marker.lifetime = ros::Duration(0.2);

    Eigen::Vector3d center = odom_pos_;
    // Marker 的 position 是圆柱几何中心；上下伸展不等时须移到顶底中点 z+(z_up-z_down)/2，不能直接用定位点。
    center(2) += 0.5 * (z_up - z_down);

    // heading 是局部三维向量变量名，不是函数名；这是“类型 变量名(构造参数)”的直接初始化写法。
    // 三个实参分别初始化 X/Y/Z：由 yaw（rad）构造世界系水平单位前向向量 (cos(yaw),sin(yaw),0)。
    // 例如 yaw=0 时 heading=(1,0,0)，yaw=π/2 时约为 (0,1,0)；Z=0 表示中心偏移不改变高度。
    // 后面 offset*heading 是从中心到前圆柱的位移，减去该位移则得到后圆柱中心。
    Eigen::Vector3d heading(std::cos(getOdomYaw()), std::sin(getOdomYaw()), 0.0);
    // front/rear 分别为前、后显示圆柱的几何中心坐标（世界系 m），后续写入 id=0/1 的 Marker.pose.position。
    // center 已包含上下厚度不对称引起的高度修正；heading.z=0，因此前后两圆柱的中心高度均为 center.z。
    Eigen::Vector3d front = center + self_double_cylinder_offset_ * heading;
    Eigen::Vector3d rear = center - self_double_cylinder_offset_ * heading;

    marker.id = 0;
    marker.pose.position.x = front(0);
    marker.pose.position.y = front(1);
    marker.pose.position.z = front(2);
    self_inflation_pub_.publish(marker);

    marker.id = 1;
    marker.pose.position.x = rear(0);
    marker.pose.position.y = rear(1);
    marker.pose.position.z = rear(2);
    self_inflation_pub_.publish(marker);
  }

  /// @brief 更新状态并统计同一状态连续调用次数，打印调用来源便于区分目标、执行和安全回调。
  void SCANReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {

    // 调用方已决定下一步状态：new_state 是要进入/继续的阶段，exec_state_ 是修改前正在执行的阶段。
    // 相同状态表示再次提交该状态请求，计数加一；换到另一个状态则从第 1 次开始计数。
    // 此计数不随定时器每次触发自动增长，也不是总失败次数；只有调用本函数才会更新。
    // 不累计某状态的历史总次数：例如连续请求 A 得到 1、2，切到 B 后为 1，再回到 A 仍从 1 开始，不接着记 3。
    if (new_state == exec_state_)
      continuously_called_times_++;
    else
      continuously_called_times_ = 1;

    // GEN_NEW_TRAJ 首次进入时计数为 1，局部规划使用非随机初值；失败后再次请求本状态，计数变大以切换随机初值。
    // state_str 仅将枚举值 0~5 映射为可读名称，供下面打印；字符串本身不控制规划行为。
    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};
    // 先记住旧状态用于日志，再更新成员 exec_state_；真正的规划/执行检查由后续 execFSMCallback() 的 switch 分支处理。
    // exec_state_ 的类型是 FSM_EXEC_STATE 枚举；int(...) 将其转成整数编号，作为 state_str 下标获取状态名称。
    // 例如旧状态 WAIT_TARGET 的编号是 1，则 pre_s=1，state_str[pre_s] 才是字符串 "WAIT_TARGET"。
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    // pos_call 记录“哪段代码发起这次状态切换请求”，由调用者手工传入，打印为日志前缀。
    // "TRIG"：目标/路线接入代码准备好参考后请求规划，如 RViz 新目标、预设点任务启动或外部 Path 输入。
    // "FSM"：execFSMCallback() 在推进任务阶段时提出请求，如局部规划成功后切到 EXEC_TRAJ。
    // "SAFETY"：checkCollisionCallback() 在安全检查中提出请求，如碰撞风险迫近且重规划失败时切到 EMERGENCY_STOP。
    // "TRIG" 不细分目标来自哪个话题或哪个导航模式，也不是实际目标数据；它只说明本次请求来自目标/路线接入处理。
    // 它不是 trigger_ 布尔成员，也不执行触发动作；实际状态已经由上句 new_state 更新。
    // 例如等待目标时收到新目标，可打印 [TRIG]: from WAIT_TARGET to GEN_NEW_TRAJ；同状态重试也会打印。
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
  }

  /// @brief 返回连续状态次数与当前状态；生成轨迹时据此决定是否启用随机初始化。
  // std::pair<T1,T2> 是标准库的二元组类型，含 first、second 两个成员；<...> 是指定成员类型的模板实参。
  // 可将 pair 理解为固定两个元素的元组；它与 std::tuple 是不同类型，后者可指定零个、两个或更多元素。
  // 此返回类型中 first 为 int 次数，second 为类内枚举 SCANReplanFSM::FSM_EXEC_STATE（不是状态名称字符串）。
  // 返回类型后面的 SCANReplanFSM::timesOfConsecutiveStateCalls() 才是成员函数名及空参数列表。
  // 这是项目自己定义的读取函数，不是 ROS 接口；只返回当前记录的快照，不计数、不切换状态、不执行规划。
  std::pair<int, SCANReplanFSM::FSM_EXEC_STATE> SCANReplanFSM::timesOfConsecutiveStateCalls()
  {
    // <int,FSM_EXEC_STATE> 指定两项类型，后面的 (...) 传入两项的值，构造一个 pair 对象并按值返回。
    // 函数体内已处于类作用域，可直接写 FSM_EXEC_STATE；调用者用 .first 取次数、.second 取枚举状态。
    return std::pair<int, FSM_EXEC_STATE>(continuously_called_times_, exec_state_);
  }

  /// @brief 将枚举状态转换为可读日志；等待目标时保持 INIT 并不必然表示故障。
  void SCANReplanFSM::printFSMExecState()
  {
    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};

    cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
  }

  /// @brief 状态机定时入口：等位姿和目标、生成/重规划、执行、切换关键点及急停恢复。
  /// EXEC_TRAJ 的普通重规划距离判断使用轨迹采样位置；执行时间结束与真实终点误差是两种判断。
  // 阅读时间线（以 mode 1 首次目标为例）：目标入口先生成全局参考 → GEN_NEW_TRAJ 生成首条局部轨迹 → EXEC_TRAJ 监视跟踪。
  // 随执行推进，EXEC_TRAJ 请求 REPLAN_TRAJ → planFromCurrentTraj() 准备衔接并更新局部轨迹 → 成功后回到 EXEC_TRAJ。
  // 两状态按调用路径区分，不能简单按“有没有旧轨迹”区分：mode 2 执行中切点、急停后恢复也会使用 GEN_NEW_TRAJ。
  // 反之，mode 1 执行中收到新目标仍请求 REPLAN_TRAJ；安全回调还可直接调用更新函数并在成功后保持 EXEC_TRAJ。
  // init() 中通过“成员函数指针 + this”注册本函数；ROS 调度回调时传入 TimerEvent 对象，下面的形参 e 引用该对象。
  // e 是本函数自己起的参数名（可理解为 event），不是特殊关键字；const ros::TimerEvent & 表示按只读引用接收事件信息。
  // 事件中 current_expected/current_real 分别记录本次回调的预期/实际调用时刻，last_real 记录上次实际调用时刻。
  // TimerEvent 是 ROS 的结构体类型，事件对象是一次回调的时间记录；区别于负责维持定时任务的 ros::Timer 句柄。
  // 例如预期 10.010 s 调用，实际 10.014 s 开始，则 current_real-current_expected 为 0.004 s 的调度延迟。
  // current_expired 是本次定时器到期并将回调加入队列的时刻；last_expected/last_expired 对应上一次的预期/入队时刻。
  // profile.last_duration 是上次回调的执行耗时（WallDuration），不是本次回调耗时，也不是定时器设定的周期。
  // 本实现保留该回调参数但未读取 e；轨迹进度依据 ros::Time::now() 和保存的起始时间计算，e 不决定状态或冻结标志。
  void SCANReplanFSM::execFSMCallback(const ros::TimerEvent &e)
  {
    // 本函数是导航流程的周期调度入口（设定周期 0.01 s），具体轨迹优化交给管理器，执行指令交给外部控制器。
    // 分支使用成员保存的最新反馈/任务数据：odom 回调更新位姿，目标入口准备参考，安全回调也可能修改 exec_state_。
    // 先补偿原地转向消耗的时间，再按轨迹时钟判断执行进度，避免“机器人没前进、参考进度却已推进”。
    updateLocalTrajTimeFreeze();

    // fsm_num 只统计本回调进入次数：每 100 次输出一次状态/缺输入提示，便于观察；与规划重试计数无关。
    // static 使其跨调用保留；名义周期下约 1 s 输出一次，但回调延迟时实际间隔会变长。
    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 100)
    {
      printFSMExecState();
      if (!have_odom_)
        cout << "no odom." << endl;
      if (!trigger_)
        cout << "wait for goal." << endl;
      fsm_num = 0;
    }

    // 每次只选择当前状态的分支；changeFSMExecState() 不会使 switch 立即重新选择分支。
    // break 退出 switch 后继续执行末尾检查/调试发布；return 则结束整个回调，等待以后再次调度。
    switch (exec_state_)
    {
    // 初始化阶段同时等待有效 odom 和目标触发；仅有位姿并不会自动规划。
    case INIT:
    {
      // have_odom_ 表示至少接收过机身状态；缺少位置/速度时不进入导航流程，此处不是检测数据是否超时。
      if (!have_odom_)
      {
        return;
      }
      // trigger_ 表示目标入口已触发导航任务；它与 have_target_ 的“目标及全局参考已就绪”含义不同。
      if (!trigger_)
      {
        return;
      }
      // 输入条件满足，离开初始化阶段；本次 break 后不会接着执行下面 WAIT_TARGET 的 case。
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    // 等待目标就绪，避免重复生成没有终点的轨迹。
    case WAIT_TARGET:
    {
      // 有触发不代表参考一定构造成功；只有 have_target_ 为 true，才安排面向该参考的局部轨迹生成。
      if (!have_target_)
        return;
      else
      {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    // 启动一轮局部轨迹生成：用于任务首次规划、mode 2 切点或允许恢复的急停退出；连续失败时切换随机初值。
    // 到这里可能已有旧局部轨迹；本状态并不以“旧轨迹不存在”为进入条件。
    // 全局参考负责给出任务走向；callReboundReplan() 从中选取前视局部目标，再结合地图构造可执行局部轨迹。
    case GEN_NEW_TRAJ:
    {
      // 准备 start_pt_/start_vel_/start_acc_：位置来自 odom，有可用旧轨迹时会借用其导数以便衔接。
      setStartStateFromOdomOrCurrentTraj();

      // Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);
      // start_yaw_(0)         = atan2(rot_x(1), rot_x(0));
      // start_yaw_(1) = start_yaw_(2) = 0.0;

      // flag_random_poly_init：构造局部规划初值时，是否在起点与局部目标之间加入带随机偏移的中间点，改变候选曲线形状。
      // 直观示意（非固定采样数值）：起点 (0,0,0.4)、局部目标 (3,0,0.4)，可尝试经过 (1.5,0.3,0.4) 或 (1.5,-0.2,0.4)。
      // 起点与局部目标不因该标志改变；候选仍须进行 B 样条优化和检查，不能直接作为执行轨迹发布。
      bool flag_random_poly_init;
      // .first 读取连续请求 GEN_NEW_TRAJ 的次数：正常首次进入为 1；失败分支再次请求同一状态后变为 2、3……。
      // 首次先用非随机候选；连续重试时换一种候选形状，避免在相似初始条件下反复优化失败，不保证一定成功。
      // 此处只选择策略，实际候选构造在后面的 callReboundReplan() → Manager::reboundReplan() 中完成。
      if (timesOfConsecutiveStateCalls().first == 1)
        flag_random_poly_init = false;
      else
        flag_random_poly_init = true;

      // callReboundReplan()：选取前方局部目标，调用管理器进行局部 B 样条规划，成功后通过 /planning/bspline 发布。
      // 参数 true 要求构造多项式初值；flag_random_poly_init 决定初值是否加入随机中间点；返回值 success 表示本次规划是否成功。
      bool success = callReboundReplan(true, flag_random_poly_init);
      if (success)
      {

        // 规划成功：清除普通规划失败计数，安排执行监视，并准备好以后进入急停时首次发布停止轨迹。
        replan_fail_count_ = 0;
        changeFSMExecState(EXEC_TRAJ, "FSM");
        flag_escape_emergency_ = true;
      }
      else
      {
        // 失败：累计一次失败并留在 GEN_NEW_TRAJ；同状态请求会增加连续次数，下次可更换随机初值重试。
        replan_fail_count_++;
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    // 执行中的重规划；成功重置失败计数，否则累计并由 finishProcess() 检查上限。
    // “重规划”表示根据更新后的起始状态/地图再算轨迹，复用同一局部规划器，并非在全局、局部之外增加第三套算法。
    // 典型请求时机：EXEC_TRAJ 的滚动更新判断、执行中接收新目标，以及安全回调尝试失败但暂不急停时。
    case REPLAN_TRAJ:
    {

      // planFromCurrentTraj()：根据当前观测/已有轨迹准备衔接状态，再调用 callReboundReplan() 更新局部轨迹，成功后发布。
      // 相比 GEN_NEW_TRAJ 直接调用局部规划入口，这里先处理已有轨迹的衔接，并在内部尝试不同初始化策略。
      // mode 1/2 还刷新全局参考，mode 3 保留外部路线参考；true 表示本轮更新成功，false 表示参考准备或局部尝试失败。
      if (planFromCurrentTraj())
      {
        // 与 GEN_NEW_TRAJ 共用失败计数：普通生成或更新任一成功，均清除已记录失败；它不是某一个函数的专属次数。
        replan_fail_count_ = 0;
        changeFSMExecState(EXEC_TRAJ, "FSM");
      }
      else
      {
        // 这里按 planFromCurrentTraj() 的最终结果只加一次；内部即使尝试过多种初值，也不在此逐次累计。
        replan_fail_count_++;
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    // 用轨迹时钟采样来决定常规重规划；关键点切换还会参考实际 odom 到点距离。
    case EXEC_TRAJ:
    {
      /* determine if need to replan */
      // local_data_ 是管理器保存的 LocalTrajData 对象：含当前局部 B 样条、速度/加速度样条、起始时刻、总时长及编号等。
      // info 是指向该现有对象的普通指针，& 取其地址，不新建/复制轨迹；info->成员 即访问 local_data_ 的对应成员。
      // 新局部轨迹由 Manager::updateTrajInfo() 写入此对象；全局多项式参考另存 global_data_，不是这里这份数据。
      LocalTrajData *info = &planner_manager_->local_data_;
      ros::Time time_now = ros::Time::now();
      // 当前 ROS 时刻减去该局部轨迹的起始时间基准，得到执行进度 t_cur（s）。
      double t_cur = (time_now - info->start_time_).toSec();
      // 将进度限制为不超过本条轨迹总时长，供后面的轨迹求值与结束判断使用；此句未限制负值下界。
      t_cur = min(info->duration_, t_cur);

      // pos 是轨迹在 t_cur 的计划位置（世界系 m），不是 odom_pos_ 测得的真实位置；两者可能有跟踪误差。
      // evaluateDeBoorT(t_cur)：按已有位置 B 样条计算轨迹开始后 t_cur 秒处的 XYZ，返回三维向量，不重新规划轨迹。
      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);

      // 仅 mode 2：还有下一个关键点，且真实位置距当前目标不足 0.5 m 时，提前切到下一目标以继续序列。
      // current_wp_ 是当前关键点索引；end_pt_ 是当前目标，active_waypoints_ 是本轮活动关键点列表。
      if (isWaypointSequenceMode() &&
          current_wp_ + 1 < (int)active_waypoints_.size() &&
          (end_pt_ - odom_pos_).norm() < 0.5)
      {
        // 先推进索引，planNextWaypoint() 才会为新的当前关键点构造全局参考；它本身不递增索引。
        current_wp_++;
        if (planNextWaypoint())
        {
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
          return;
        }
        // 参考准备失败也转 GEN_NEW_TRAJ 并计一次失败；此处保留现有恢复流程，不表示下一关键点参考一定成功。
        replan_fail_count_++;
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        return;
      }

      /* && (end_pt_ - pos).norm() < 0.5 */
      // 剩余局部轨迹时间不足 0.01 s 时进入结束/切点处理；此条件未检查真实位置已到达目标。
      if (t_cur > info->duration_ - 1e-2)
      {
        // mode 2 若仍有后续点，则继续序列；这是一条独立于上面“实际距离不足 0.5 m”的切点路径。
        if (isWaypointSequenceMode() && current_wp_ + 1 < (int)active_waypoints_.size())
        {
          current_wp_++;
          if (planNextWaypoint())
          {
            changeFSMExecState(GEN_NEW_TRAJ, "FSM");
            return;
          }
          replan_fail_count_++;
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
          return;
        }

        // mode 2 无后续点时清理活动序列；其余模式直接进入下面的任务目标清除流程。
        if (isWaypointSequenceMode())
        {
          active_waypoints_.clear();
          current_wp_ = 0;
        }

        // 本分支把当前任务目标标记为结束，回到等待；实际到点精度仍需结合 odom 与外部控制器判断。
        have_target_ = false;

        changeFSMExecState(WAIT_TARGET, "FSM");
        return;
      }
      else if ((end_pt_ - pos).norm() < no_replan_thresh_)
      {
        // 计划位置已距目标不足 no_replan_thresh_（默认 0.1 m），保留当前局部轨迹，不触发本处普通重规划。
        // cout << "near end" << endl;
        return;
      }
      else if ((info->start_pos_ - pos).norm() < replan_thresh_)
      {
        // 计划位置距本条局部轨迹起点仍不足 replan_thresh_（默认 1.0 m），先沿现有轨迹执行，避免过早反复规划。
        // 此处比较起点与采样点的直线距离，不是沿曲线累计走过的路程。
        // cout << "near start" << endl;
        return;
      }
      else
      {
        // 未到结束时间、未接近目标且已离开局部起点足够远，安排下一次局部重规划，滚动更新后续轨迹。
        // 上面两种 return 只抑制本处普通重规划，独立安全回调仍可因碰撞风险触发重规划或急停。
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
      break;
    }

    // 只在进入时发布停止轨迹；低速后按 fail_safe 和 need_hover_stop_ 决定后续状态。
    case EMERGENCY_STOP:
    {

      // flag_escape_emergency_ 是急停首次处理标志；为 true 时发布停在当前 odom 位置的轨迹，避免每周期重复发布。
      if (flag_escape_emergency_) // Avoiding repeated calls
      {
        callEmergencyStop(odom_pos_);
      }
      else
      {
        // 已发布停止轨迹后，等观测速度范数小于 0.1 m/s；enable_fail_safe_ 决定是否允许自动退出急停。
        // need_hover_stop_=false：允许为原任务重新规划；为 true：要求清除旧任务，等待新目标。
        if (enable_fail_safe_ && !need_hover_stop_ && odom_vel_.norm() < 0.1)
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        else if (enable_fail_safe_ && need_hover_stop_ && odom_vel_.norm() < 0.1)
        {
          ROS_INFO("Exiting EMERGENCY_STOP. Switching to WAIT_TARGET. Need a new target point.");
          need_hover_stop_ = false;
          have_target_ = false;
          trigger_ = false;
          changeFSMExecState(WAIT_TARGET, "EMERGENCY_EXIT");
        }
      }

      // 标记本次急停的首次处理已完成；后续回调进入上面的 else 检查是否满足退出条件。
      flag_escape_emergency_ = false;
      break;
    }
    }

    // 只有未提前 return 的路径才到这里：普通规划失败计数达到上限时，finishProcess() 安排急停并等待新目标。
    finishProcess();

    // 发布调试消息 /planning/data_display；当前这里只刷新时间戳，不是向机器人发送运动控制命令。
    data_disp_.header.stamp = ros::Time::now();
    // data_disp_ 是 scan_planner::DataDisp 消息（定义见 msg/DataDisp.msg），包含 header 和五个 float64 字段 a~e。
    // 当前源码未给 a~e 赋业务数据，因此保持消息默认值 0.0；此处主要发送更新后的时间戳，未携带轨迹或状态枚举。
    // data_disp_pub_ 是发布器句柄，publish(data_disp_) 将上述消息发到初始化时绑定的 /planning/data_display。
    data_disp_pub_.publish(data_disp_);
  }

  /// @brief 累计失败达到上限时设置 need_hover_stop_ 并进入急停，准备等待新的导航目标。
  void SCANReplanFSM::finishProcess()
  {
    if (replan_fail_count_ >= max_replan_fail_count_)
    {
      ROS_WARN("Replan failed %d times. Emergency stop and wait for a new target.", replan_fail_count_);
      replan_fail_count_ = 0;
      need_hover_stop_ = true;
      flag_escape_emergency_ = true;
      changeFSMExecState(EMERGENCY_STOP, "finishProcess");
    }
  }

  /// @brief 为当前目标重新组织参考与起始导数，调用优化；失败时尝试更换初始化方式。
  /// mode 1/2 以 odom 位置更新起点；mode 3 使用当前轨迹采样的起始状态。
  // 三种模式在此复用局部发布链，但参考处理不同：mode 1/2 刷新当前起点到目标的全局参考，mode 3 保留外部路线参考。
  bool SCANReplanFSM::planFromCurrentTraj()
  {
    LocalTrajData *info = &planner_manager_->local_data_;
    ros::Time time_now = ros::Time::now();
    double t_cur = (time_now - info->start_time_).toSec();
    t_cur = std::min(std::max(t_cur, 0.0), info->duration_);

    //cout << "info->velocity_traj_=" << info->velocity_traj_.get_control_points() << endl;

    if (navi_mode_ == NAVI_MODE::REFERENCE_PATH)
    {
      // 分别对旧轨迹的位置、速度、加速度样条在同一相对时刻 t_cur 求值，作为新局部轨迹的起始状态以便衔接。
      // evaluateDeBoorT() 只计算调用它的样条的值：这里依次返回位置（m）、速度（m/s）、加速度（m/s²）。
      start_pt_ = info->position_traj_.evaluateDeBoorT(t_cur);
      start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
      start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

      // 按顺序尝试局部规划策略，前一种失败才进入下一种；参数是初始化选项，success 才是本次规划的返回结果。
      // 第 1 次：不主动要求重建多项式初值，通常利用旧轨迹构造候选；不引入随机中间点。
      // 例外：have_new_target_ 为 true 或管理器要求重新初始化时，即使首参为 false，也可能构造多项式初值。
      bool success = callReboundReplan(false, false);
      if (!success)
      {
        // 第 2 次：放弃继续使用旧轨迹初值的策略，明确要求重建非随机多项式候选，再尝试局部优化。
        success = callReboundReplan(true, false);
        if (!success)
        {
          // 第 3 次：重建带随机中间点的多项式候选，换一种初始曲线形状再尝试。
          success = callReboundReplan(true, true);
          // 三次均失败，向调用方报告本轮更新失败；成功的尝试已在 callReboundReplan() 中发布轨迹。
          if (!success)
            return false;
        }
      }

      // 能到这里说明至少有一次成功；较早的尝试成功后，后续备用策略不会再执行。
      return true;
    }

    start_pt_ = odom_pos_;
    start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
    start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

    const Eigen::Vector2d to_goal = end_pt_.head<2>() - odom_pos_.head<2>();
    if (to_goal.norm() > 1e-3 && start_vel_.head<2>().dot(to_goal) < 0.0)
    {
      start_vel_.setZero();
      start_acc_.setZero();
    }

    if (!planner_manager_->planGlobalTraj(
            start_pt_,
            start_vel_,
            start_acc_,
            end_pt_,
            Eigen::Vector3d::Zero(),
            Eigen::Vector3d::Zero()))
    {
      ROS_ERROR("[navi_mode=%d] Unable to refresh global trajectory from odom to current target.", navi_mode_);
      return false;
    }

    if (!adjustGlobalTargetIfOccupied())
      return false;

    bool success = callReboundReplan(true, false);
    if (!success)
    {
      success = callReboundReplan(true, true);
      if (!success)
        return false;
    }

    return true;
  }

  /// @brief 统一确定新规划的起点位置、速度、加速度，处理已有轨迹与当前观测之间的衔接。
  void SCANReplanFSM::setStartStateFromOdomOrCurrentTraj()
  {
    start_pt_ = odom_pos_;
    start_vel_ = odom_vel_;
    start_acc_.setZero();

    LocalTrajData *info = &planner_manager_->local_data_;
    if (info->start_time_.toSec() < 1e-5 || info->duration_ <= 1e-5)
      return;

    const double raw_t_cur = (ros::Time::now() - info->start_time_).toSec();
    if (raw_t_cur < -1e-3 || raw_t_cur > info->duration_ + 0.2)
      return;

    const double t_cur = std::min(std::max(raw_t_cur, 0.0), info->duration_);
    start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
    start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

    const Eigen::Vector2d to_goal = end_pt_.head<2>() - odom_pos_.head<2>();
    if (to_goal.norm() > 1e-3 && start_vel_.head<2>().dot(to_goal) < 0.0)
    {
      start_vel_.setZero();
      start_acc_.setZero();
    }
  }

  /// @brief 沿当前轨迹检查带 yaw 的膨胀占用，先尝试重规划，失败后按碰撞剩余时间选择急停或重规划。
  /// 当前位置还在前 2/3 时间段时检查到该边界；进入后段后检查剩余段，采样步长为 0.01 s。
  void SCANReplanFSM::checkCollisionCallback(const ros::TimerEvent &e)
  {
    updateLocalTrajTimeFreeze();

    LocalTrajData *info = &planner_manager_->local_data_;
    auto map = planner_manager_->grid_map_;

    if (exec_state_ == WAIT_TARGET || info->start_time_.toSec() < 1e-5)
      return;

    /* ---------- check trajectory ---------- */
    // 安全回调内部的轨迹采样间隔（s），与安全回调 0.05 s 的触发周期是两个尺度。
    constexpr double time_step = 0.01;
    double t_cur = (ros::Time::now() - info->start_time_).toSec();
    double t_2_3 = info->duration_ * 2 / 3;
    for (double t = t_cur; t < info->duration_; t += time_step)
    {
      if (t_cur < t_2_3 && t >= t_2_3) // If t_cur < t_2_3, only the first 2/3 partition of the trajectory is considered valid and will get checked.
        break;

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t);
      Eigen::Vector3d pos_next = info->position_traj_.evaluateDeBoorT(std::min(t + time_step, info->duration_));
      if (map->getInflateOccupancy(pos, estimateYawFromSegment(pos, pos_next)))
      {
        // 安全回调直接尝试更新并发布局部轨迹：重规划是一个动作，发生该动作不一定先切到 REPLAN_TRAJ 状态。
        if (planFromCurrentTraj()) // Make a chance
        {
          changeFSMExecState(EXEC_TRAJ, "SAFETY");
          return;
        }
        else
        {
          if (t - t_cur < emergency_time_) // 0.8s of emergency time
          {
            ROS_WARN("Suddenly discovered obstacles. emergency stop! time=%f", t - t_cur);
            changeFSMExecState(EMERGENCY_STOP, "SAFETY");
          }
          else
          {
            //ROS_WARN("current traj in collision, replan.");
            changeFSMExecState(REPLAN_TRAJ, "SAFETY");
          }
          return;
        }
        break;
      }
    }
  }

  /// @brief 选取局部目标并调用 Manager；成功后把控制点、knots、阶数和起始时间写入 Bspline 消息。
  /// 只有规划返回成功才发布这次新轨迹，并更新显示。
  bool SCANReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
  {

    getLocalTarget();

    // 目标刚更新或调用方要求多项式初始化，任一为 true 就重建初值；此条件不决定能否调用规划器。
    // 传给 Manager 的 flag_polyInit = have_new_target_ || flag_use_poly_init：
    // 新目标/新路线/预设关键点就绪时由前者触发；GEN_NEW_TRAJ、mode 1/2 更新及 mode 3 备用重试由后者触发。
    // 安全回调通过 planFromCurrentTraj() 复用相同策略，不能仅凭该参数为 true 就判断“刚刚发现障碍”。
    bool plan_success =
        planner_manager_->reboundReplan(start_pt_, start_vel_, start_acc_, local_target_pt_, local_target_vel_, (have_new_target_ || flag_use_poly_init), flag_randomPolyTraj);
    // 完成这一次“局部轨迹规划尝试”后消费更新提示，不论是否成功；不是在前面的全局参考生成后立即清除。
    // have_target_ 不在这里清除，已有目标/参考及后续局部重规划仍可继续。
    have_new_target_ = false;

    cout << "final_plan_success=" << plan_success << endl;

    if (plan_success)
    {

      auto info = &planner_manager_->local_data_;

      /* publish traj */
      scan_planner::Bspline bspline;
      bspline.order = 3;
      bspline.start_time = info->start_time_;
      bspline.traj_id = info->traj_id_;

      Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
      bspline.pos_pts.reserve(pos_pts.cols());
      for (int i = 0; i < pos_pts.cols(); ++i)
      {
        geometry_msgs::Point pt;
        pt.x = pos_pts(0, i);
        pt.y = pos_pts(1, i);
        pt.z = pos_pts(2, i);
        bspline.pos_pts.push_back(pt);
      }

      Eigen::VectorXd knots = info->position_traj_.getKnot();
      bspline.knots.reserve(knots.rows());
      for (int i = 0; i < knots.rows(); ++i)
      {
        bspline.knots.push_back(knots(i));
      }

      bspline_pub_.publish(bspline);

      visualization_->displayOptimalTraj(info->position_traj_, 0);
    }

    return plan_success;
  }

  /// @brief 调用 Manager 构造固定位置轨迹，再按与普通轨迹相同的 Bspline 接口发布。
  bool SCANReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
  {

    planner_manager_->EmergencyStop(stop_pos);

    auto info = &planner_manager_->local_data_;

    /* publish traj */
    scan_planner::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = info->start_time_;
    bspline.traj_id = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    bspline.pos_pts.reserve(pos_pts.cols());
    for (int i = 0; i < pos_pts.cols(); ++i)
    {
      geometry_msgs::Point pt;
      pt.x = pos_pts(0, i);
      pt.y = pos_pts(1, i);
      pt.z = pos_pts(2, i);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
    bspline.knots.reserve(knots.rows());
    for (int i = 0; i < knots.rows(); ++i)
    {
      bspline.knots.push_back(knots(i));
    }

    bspline_pub_.publish(bspline);

    return true;
  }

  /// @brief 将起点投影到全局参考，再沿参考累积 planning_horizon_ 距离选局部目标。
  /// 局部目标不可用时沿参考前后搜索；接近终点时将目标速度置零，其他位置沿用参考速度。
  void SCANReplanFSM::getLocalTarget()
  {
    // 全局参考到局部规划的连接：先寻找参考上靠近 start_pt_ 的位置，再向前累计约 planning_horizon_（默认 3.5 m）取目标。
    // 前视距离沿参考采样点累加；终点更近时使用终点，候选被占用时还会沿参考调整，所以实际距离不固定为 3.5 m。
    // 局部目标以参考轨迹的累积空间距离选取；时间采样步长由速度与前视距离估计。
    const double max_vel = planner_manager_->pp_.max_vel_;
    const double max_acc = planner_manager_->pp_.max_acc_;
    const double duration = planner_manager_->global_data_.global_duration_;
    double t_step = max_vel > 1e-6 ? planning_horizon_ / 20.0 / max_vel : 0.01;
    t_step = std::max(t_step, 0.01);

    double t_proj = 0.0;
    double min_dist_to_start = 9999.0;
    for (double t = 0.0; t < duration; t += t_step)
    {
      Eigen::Vector3d pos_t = planner_manager_->global_data_.getPosition(t);
      double dist_to_start = (pos_t - start_pt_).norm();
      if (dist_to_start < min_dist_to_start)
      {
        min_dist_to_start = dist_to_start;
        t_proj = t;
      }
    }

    double target_t = duration;
    double total_dist = 0.0;
    bool target_found = false;
    Eigen::Vector3d prev_pos = planner_manager_->global_data_.getPosition(t_proj);
    local_target_pt_ = end_pt_;

    for (double t = t_proj; t < duration; t += t_step)
    {
      Eigen::Vector3d pos_t = planner_manager_->global_data_.getPosition(t);
      total_dist += (pos_t - prev_pos).norm();
      if (total_dist >= planning_horizon_)
      {
        local_target_pt_ = pos_t;
        target_t = t;
        target_found = true;
        break;
      }
      prev_pos = pos_t;
    }
    planner_manager_->global_data_.last_progress_time_ = target_found ? target_t : duration;

    auto targetOccupancy = [&](const Eigen::Vector3d &pt) {
      return planner_manager_->grid_map_->getInflateOccupancy(pt, estimateYawFromSegment(odom_pos_, pt));
    };

    if (targetOccupancy(local_target_pt_) != 0)
    {
      bool found_free_target = false;
      double adjusted_t = target_t;

      for (double dt = 0.0; dt <= planner_manager_->global_data_.global_duration_; dt += t_step)
      {
        double t_forward = target_t + dt;
        if (t_forward <= planner_manager_->global_data_.global_duration_)
        {
          Eigen::Vector3d pt = planner_manager_->global_data_.getPosition(t_forward);
          if (targetOccupancy(pt) == 0)
          {
            local_target_pt_ = pt;
            adjusted_t = t_forward;
            found_free_target = true;
            break;
          }
        }

        double t_backward = target_t - dt;
        if (t_backward >= std::max(0.0, t_proj))
        {
          Eigen::Vector3d pt = planner_manager_->global_data_.getPosition(t_backward);
          if (targetOccupancy(pt) == 0)
          {
            local_target_pt_ = pt;
            adjusted_t = t_backward;
            found_free_target = true;
            break;
          }
        }
      }

      if (found_free_target)
      {
        // 目标调整可能频繁发生；此处仅将提示限制为约每 1.0 s 一条，不限制实际调整次数。
        ROS_WARN_THROTTLE(1.0, "Local target in collision, adjusted to a nearby collision-free point.");
        target_t = adjusted_t;
      }
      else
      {
        // 按 1.0 s 间隔限制这处失败提示；只控制打印，不代表每秒才尝试一次搜索。
        ROS_WARN_THROTTLE(1.0, "Local target in collision and no nearby collision-free target was found.");
      }
    }

    if ((end_pt_ - local_target_pt_).norm() < (max_vel * max_vel) / (2 * max_acc))
    {
      // local_target_vel_ = (end_pt_ - init_pt_).normalized() * planner_manager_->pp_.max_vel_ * (( end_pt_ - local_target_pt_ ).norm() / ((planner_manager_->pp_.max_vel_*planner_manager_->pp_.max_vel_)/(2*planner_manager_->pp_.max_acc_)));
      // cout << "A" << endl;
      local_target_vel_ = Eigen::Vector3d::Zero();
    }
    else
    {
      local_target_vel_ = planner_manager_->global_data_.getVelocity(target_t);
      if (local_target_vel_.norm() > max_vel)
        local_target_vel_ = local_target_vel_.normalized() * max_vel;
      // cout << "AA" << endl;
    }
  }

} // namespace scan_planner
