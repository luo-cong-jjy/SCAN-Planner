/// @file
/// 规划调试显示库：由 FSM/Manager 调用，把目标、参考、初始轨迹、A* 引导及最优轨迹转换为 Marker。
/// 构造函数使用规划器私有句柄，因此主要输出在 /scan_planner_node 下；显示不反馈控制器。
/// 不同显示接口的 frame_id 需逐个核对，不能仅凭 Marker 名称推断坐标系。
#include <traj_utils/planning_visualization.h>
#include <cmath>
#include <limits>
#include <std_msgs/ColorRGBA.h>

using std::cout;
using std::endl;
namespace scan_planner
{
  // 有参构造函数的类外定义：前一个 PlanningVisualization 是所属类，后一个同名部分是构造函数名。
  // FSM 执行 new PlanningVisualization(nh) 时立即调用这里，保存私有句柄并注册下面五路 Marker 输出。
  // 此时只建立发布接口；具体目标、路径等内容在后续 display*() 调用时才发送。
  PlanningVisualization::PlanningVisualization(ros::NodeHandle &nh)
  {
    // 复制 ROS 句柄的管理状态；node 与调用方 nh 是不同句柄对象，复用节点资源，不是复制一个新 ROS 节点。
    node = nh;

    // 运行连接（输出入口）：私有规划目标 Marker，默认 /scan_planner_node/goal_point。
    // 队列容量：2 条消息，不是发布/规划频率。
    goal_point_pub = nh.advertise<visualization_msgs::Marker>("goal_point", 2);
    // 运行连接（输出入口）：私有全局多项式参考 Marker，默认 /scan_planner_node/global_list。
    // 队列容量：2 条消息，不是发布/规划频率。
    global_list_pub = nh.advertise<visualization_msgs::Marker>("global_list", 2);
    // 运行连接（输出入口）：私有局部初值 Marker，默认 /scan_planner_node/init_list，用于与优化结果
    // 比较。
    // 队列容量：2 条消息，不是发布/规划频率。
    init_list_pub = nh.advertise<visualization_msgs::Marker>("init_list", 2);
    // 运行连接（输出入口）：私有优化结果 Marker，默认 /scan_planner_node/optimal_list。
    // 队列容量：2 条消息，不是发布/规划频率。
    optimal_list_pub = nh.advertise<visualization_msgs::Marker>("optimal_list", 2);
    // 运行连接（输出入口）：私有碰撞段 A* 绕行引导 Marker，默认 /scan_planner_node/a_star_list，不
    // 是直接执行的控制轨迹。
    // 队列容量：20 条消息，不是发布/规划频率。
    a_star_list_pub = nh.advertise<visualization_msgs::Marker>("a_star_list", 20);
  }

  // // real ids used: {id, id+1000}
  /// @brief 把离散空间点同时画成球列表和连线，用 id/id+1000 区分两类 Marker。
  void PlanningVisualization::displayMarkerList(ros::Publisher &pub, const vector<Eigen::Vector3d> &list, double scale,
                                                Eigen::Vector4d color, int id)
  {
    visualization_msgs::Marker sphere, line_strip;
    sphere.header.frame_id = line_strip.header.frame_id = "world";
    sphere.header.stamp = line_strip.header.stamp = ros::Time::now();
    sphere.type = visualization_msgs::Marker::SPHERE_LIST;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    sphere.action = line_strip.action = visualization_msgs::Marker::ADD;
    sphere.id = id;
    line_strip.id = id + 1000;

    sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
    sphere.color.r = line_strip.color.r = color(0);
    sphere.color.g = line_strip.color.g = color(1);
    sphere.color.b = line_strip.color.b = color(2);
    sphere.color.a = line_strip.color.a = color(3) > 1e-5 ? color(3) : 1.0;
    sphere.scale.x = scale;
    sphere.scale.y = scale;
    sphere.scale.z = scale;
    line_strip.scale.x = scale / 2;
    geometry_msgs::Point pt;
    for (int i = 0; i < int(list.size()); i++)
    {
      pt.x = list[i](0);
      pt.y = list[i](1);
      pt.z = list[i](2);
      sphere.points.push_back(pt);
      line_strip.points.push_back(pt);
    }
    pub.publish(sphere);
    pub.publish(line_strip);
  }

  // real ids used: {id, id+1}
  /// @brief 为路径创建 MarkerArray，按调用接口设置颜色、尺度和 frame，返回给上层发布。
  void PlanningVisualization::generatePathDisplayArray(visualization_msgs::MarkerArray &array,
                                                       const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id)
  {
    visualization_msgs::Marker sphere, line_strip;
    sphere.header.frame_id = line_strip.header.frame_id = "map";
    sphere.header.stamp = line_strip.header.stamp = ros::Time::now();
    sphere.type = visualization_msgs::Marker::SPHERE_LIST;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    sphere.action = line_strip.action = visualization_msgs::Marker::ADD;
    sphere.id = id;
    line_strip.id = id + 1;

    sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
    sphere.color.r = line_strip.color.r = color(0);
    sphere.color.g = line_strip.color.g = color(1);
    sphere.color.b = line_strip.color.b = color(2);
    sphere.color.a = line_strip.color.a = color(3) > 1e-5 ? color(3) : 1.0;
    sphere.scale.x = scale;
    sphere.scale.y = scale;
    sphere.scale.z = scale;
    line_strip.scale.x = scale / 3;
    geometry_msgs::Point pt;
    for (int i = 0; i < int(list.size()); i++)
    {
      pt.x = list[i](0);
      pt.y = list[i](1);
      pt.z = list[i](2);
      sphere.points.push_back(pt);
      line_strip.points.push_back(pt);
    }
    array.markers.push_back(sphere);
    array.markers.push_back(line_strip);
  }

  // real ids used: {1000*id ~ (arrow nums)+1000*id}
  /// @brief 把位置/方向数据组织成箭头数组，用于显示局部几何方向。
  void PlanningVisualization::generateArrowDisplayArray(visualization_msgs::MarkerArray &array,
                                                        const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id)
  {
    visualization_msgs::Marker arrow;
    arrow.header.frame_id = "map";
    arrow.header.stamp = ros::Time::now();
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.action = visualization_msgs::Marker::ADD;

    // geometry_msgs::Point start, end;
    // arrow.points

    arrow.color.r = color(0);
    arrow.color.g = color(1);
    arrow.color.b = color(2);
    arrow.color.a = color(3) > 1e-5 ? color(3) : 1.0;
    arrow.scale.x = scale;
    arrow.scale.y = 2 * scale;
    arrow.scale.z = 2 * scale;

    geometry_msgs::Point start, end;
    for (int i = 0; i < int(list.size() / 2); i++)
    {
      // arrow.color.r = color(0) / (1+i);
      // arrow.color.g = color(1) / (1+i);
      // arrow.color.b = color(2) / (1+i);

      start.x = list[2 * i](0);
      start.y = list[2 * i](1);
      start.z = list[2 * i](2);
      end.x = list[2 * i + 1](0);
      end.y = list[2 * i + 1](1);
      end.z = list[2 * i + 1](2);
      arrow.points.clear();
      arrow.points.push_back(start);
      arrow.points.push_back(end);
      arrow.id = i + id * 1000;

      array.markers.push_back(arrow);
    }
  }

  /// @brief 发布单个目标球体，编号可区分关键点；显示点不代表目标已通过占用检查。
  void PlanningVisualization::displayGoalPoint(Eigen::Vector3d goal_point, Eigen::Vector4d color, const double scale, int id)
  {
    // lifetime 默认 0，表示 RViz 不按时长自动删除；同一命名空间下，不同 id 可共存，相同 id 更新已有标记。
    visualization_msgs::Marker sphere;
    sphere.header.frame_id = "world";
    sphere.header.stamp = ros::Time::now();
    sphere.type = visualization_msgs::Marker::SPHERE;
    sphere.action = visualization_msgs::Marker::ADD;
    sphere.id = id;

    sphere.pose.orientation.w = 1.0;
    sphere.color.r = 1.0;
    sphere.color.g = 1.0;
    sphere.color.b = 1.0;
    sphere.color.a = color(3);
    sphere.scale.x = scale;
    sphere.scale.y = scale;
    sphere.scale.z = scale;
    sphere.pose.position.x = goal_point(0);
    sphere.pose.position.y = goal_point(1);
    sphere.pose.position.z = goal_point(2);

    goal_point_pub.publish(sphere);
  }

  /// @brief 显示全局多项式参考采样点，用来与局部优化结果比较。
  void PlanningVisualization::displayGlobalPathList(vector<Eigen::Vector3d> init_pts, const double scale, int id)
  {

    if (global_list_pub.getNumSubscribers() == 0)
    {
      return;
    }

    Eigen::Vector4d color(0, 0.5, 0.5, 1);
    displayMarkerList(global_list_pub, init_pts, scale, color, id);
  }

  /// @brief 显示本次局部优化之前的初始化路径，便于区分初值问题和优化问题。
  void PlanningVisualization::displayInitPathList(vector<Eigen::Vector3d> init_pts, const double scale, int id)
  {

    if (init_list_pub.getNumSubscribers() == 0)
    {
      return;
    }

    Eigen::Vector4d color(0, 0, 1, 1);
    displayMarkerList(init_list_pub, init_pts, scale, color, id);
  }

  /// @brief 显示优化控制点列表；控制多边形与样条求值曲线是不同对象。
  void PlanningVisualization::displayOptimalList(Eigen::MatrixXd optimal_pts, int id)
  {

    if (optimal_list_pub.getNumSubscribers() == 0)
    {
      return;
    }

    vector<Eigen::Vector3d> list;
    for (int i = 0; i < optimal_pts.cols(); i++)
    {
      Eigen::Vector3d pt = optimal_pts.col(i).transpose();
      list.push_back(pt);
    }
    Eigen::Vector4d color(1, 0, 0, 1);
    displayMarkerList(optimal_list_pub, list, 0.15, color, id);
  }

  /// @brief 按样条求值采样并绘制最优轨迹，使用导数等信息组织显示。
  void PlanningVisualization::displayOptimalTraj(UniformBspline position_traj, int id)
  {
    if (optimal_list_pub.getNumSubscribers() == 0)
    {
      return;
    }

    const double duration = position_traj.getTimeSum();
    if (duration < 1e-6)
    {
      return;
    }

    UniformBspline velocity_traj = position_traj.getDerivative();
    const int sample_num = std::max(2, static_cast<int>(std::ceil(duration / 0.1)) + 1);

    std::vector<Eigen::Vector3d> points;
    std::vector<double> speeds;
    points.reserve(sample_num);
    speeds.reserve(sample_num);

    double min_speed = std::numeric_limits<double>::max();
    double max_speed = 0.0;
    for (int i = 0; i < sample_num; ++i)
    {
      const double t = duration * static_cast<double>(i) / static_cast<double>(sample_num - 1);
      Eigen::Vector3d pos = position_traj.evaluateDeBoorT(t);
      const double speed = velocity_traj.evaluateDeBoorT(t).norm();

      points.push_back(pos);
      speeds.push_back(speed);
      min_speed = std::min(min_speed, speed);
      max_speed = std::max(max_speed, speed);
    }

    visualization_msgs::Marker sphere, line_strip;
    sphere.header.frame_id = line_strip.header.frame_id = "world";
    sphere.header.stamp = line_strip.header.stamp = ros::Time::now();
    sphere.type = visualization_msgs::Marker::SPHERE_LIST;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    sphere.action = line_strip.action = visualization_msgs::Marker::ADD;
    sphere.id = id;
    line_strip.id = id + 1000;
    sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
    sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.08;
    line_strip.scale.x = 0.08;

    const double speed_range = std::max(1e-6, max_speed - min_speed);
    for (size_t i = 0; i < points.size(); ++i)
    {
      geometry_msgs::Point pt;
      pt.x = points[i](0);
      pt.y = points[i](1);
      pt.z = points[i](2);
      sphere.points.push_back(pt);
      line_strip.points.push_back(pt);

      const double ratio = std::min(1.0, std::max(0.0, (speeds[i] - min_speed) / speed_range));
      std_msgs::ColorRGBA color;
      color.a = 1.0;
      color.r = 1.0;
      color.g = 1.0 - ratio;
      color.b = 0.0;
      sphere.colors.push_back(color);
      line_strip.colors.push_back(color);
    }

    optimal_list_pub.publish(sphere);
    optimal_list_pub.publish(line_strip);
  }

  /// @brief 显示各碰撞段对应的 A* 绕行引导；这些路径用于生成优化约束，不直接作为控制命令。
  void PlanningVisualization::displayAStarList(std::vector<std::vector<Eigen::Vector3d>> a_star_paths, int id /* = Eigen::Vector4d(0.5,0.5,0,1)*/)
  {

    if (a_star_list_pub.getNumSubscribers() == 0)
    {
      return;
    }

    int i = 0;
    vector<Eigen::Vector3d> list;

    Eigen::Vector4d color = Eigen::Vector4d(0.5 + ((double)rand() / RAND_MAX / 2), 0.5 + ((double)rand() / RAND_MAX / 2), 0, 1); // make the A star paths different every time.
    double scale = 0.05 + (double)rand() / RAND_MAX / 10;

    // for ( int i=0; i<10; i++ )
    // {
    //   //Eigen::Vector4d color(1,1,0,0);
    //   displayMarkerList(a_star_list_pub, list, scale, color, id+i);
    // }

    for (auto block : a_star_paths)
    {
      list.clear();
      for (auto pt : block)
      {
        list.push_back(pt);
      }
      //Eigen::Vector4d color(0.5,0.5,0,1);
      displayMarkerList(a_star_list_pub, list, scale, color, id + i); // real ids used: [ id ~ id+a_star_paths.size() ]
      i++;
    }
  }

  /// @brief 将输入向量逐项画成箭头，便于检查优化方向或其他调试数据。
  void PlanningVisualization::displayArrowList(ros::Publisher &pub, const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id)
  {
    visualization_msgs::MarkerArray array;
    // clear
    pub.publish(array);

    generateArrowDisplayArray(array, list, scale, color, id);

    pub.publish(array);
  }

  // PlanningVisualization::
} // namespace scan_planner
