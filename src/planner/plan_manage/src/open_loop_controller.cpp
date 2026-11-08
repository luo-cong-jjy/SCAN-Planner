/// @file
/// 开环轨迹采样节点：由 controller_mode=open_loop 启动，替代闭环控制器及平面运动学仿真。
/// 直接把位置/导数样条采样为 Odometry，可显示三维位置变化；不输出真实反馈跟踪误差模型。
/// 使用 Bspline 消息的 start_time 计算执行经过时间，与闭环控制器的 exec_time 策略不同。
#include "bspline_opt/uniform_bspline.h"
#include "scan_planner/Bspline.h"

#include <Eigen/Eigen>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

using scan_planner::UniformBspline;

namespace
{
// 直接发布轨迹采样的机身状态，默认 /quad_0/body_pose，不经过 /cmd_vel 和平面速度积分。
ros::Publisher odom_pub;

// 尚无轨迹时为 false，发布初始状态；收到有效样条后为 true。
bool receive_traj = false;
// 当前消息重建出的 [位置,速度,加速度] 样条，用于直接采样三维运动及切向朝向。
std::vector<UniformBspline> traj;
// 消息位置样条的有效时长（s），用于截断采样时间。
double traj_duration = 0.0;
// 直接使用消息 start_time 作为计时原点，不采用闭环控制器的独立 exec_time。
ros::Time start_time;
// 最近收到的消息编号，用于接收日志。
int traj_id = 0;

// 输出机身位置的参考系，main() 从私有配置读取，缺省 world。
std::string frame_id;
// 输出机身子坐标系名，缺省 quadruped；与闭环仿真的默认 base 名称不同。
std::string child_frame_id;
// 最近有效水平切向朝向（rad），私有 init_yaw 缺省 0；水平速度很小时保持上次值。
double last_yaw = 0.0;
// 更新切向朝向的最低水平速度（m/s），私有参数缺省 0.05，避免低速方向抖动。
double yaw_min_speed = 0.05;
// 私有配置缺省 true；轨迹结束后继续发布终点、零速度，false 时超过时长则不再发布。
bool hold_final_position = true;
// 最近发布的位置（世界系 m）；main() 用 /init_x/y/z 设置初始值，之后跟随轨迹采样。
Eigen::Vector3d current_pos = Eigen::Vector3d::Zero();
// 最近发布的世界速度（m/s），起始为零，结束保持终点时恢复零速度。
Eigen::Vector3d current_vel = Eigen::Vector3d::Zero();

/// @brief 将位置、世界系速度和水平朝向封装为 Odometry，frame_id/child_frame_id 从私有配置读取。
void publishState(const ros::Time& stamp, const Eigen::Vector3d& pos, const Eigen::Vector3d& vel,
                  double yaw, double yaw_rate)
{
  nav_msgs::Odometry odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = frame_id;
  odom.child_frame_id = child_frame_id;

  odom.pose.pose.position.x = pos.x();
  odom.pose.pose.position.y = pos.y();
  odom.pose.pose.position.z = pos.z();
  odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

  odom.twist.twist.linear.x = vel.x();
  odom.twist.twist.linear.y = vel.y();
  odom.twist.twist.linear.z = vel.z();
  odom.twist.twist.angular.z = yaw_rate;

  odom_pub.publish(odom);
}

/// @brief 检查基本消息字段并重建位置样条，knots 使用消息原值；无效消息保留已有轨迹。
bool parseBspline(const scan_planner::BsplineConstPtr& msg, UniformBspline& pos_traj)
{
  if (msg->pos_pts.empty() || msg->knots.empty() || msg->order <= 0)
  {
    ROS_WARN("[open_loop_controller] Ignore invalid bspline.");
    return false;
  }

  Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());
  for (size_t i = 0; i < msg->pos_pts.size(); ++i)
  {
    pos_pts(0, i) = msg->pos_pts[i].x;
    pos_pts(1, i) = msg->pos_pts[i].y;
    pos_pts(2, i) = msg->pos_pts[i].z;
  }

  Eigen::VectorXd knots(msg->knots.size());
  for (size_t i = 0; i < msg->knots.size(); ++i)
    knots(i) = msg->knots[i];

  pos_traj = UniformBspline(pos_pts, msg->order, 0.1);
  pos_traj.setKnot(knots);
  return true;
}

/// @brief 接收并保存新轨迹及其一、二阶导数，记录消息指定的起始时刻、编号和时长。
void bsplineCallback(const scan_planner::BsplineConstPtr& msg)
{
  UniformBspline pos_traj;
  if (!parseBspline(msg, pos_traj))
    return;

  traj.clear();
  traj.push_back(pos_traj);
  traj.push_back(traj[0].getDerivative());
  traj.push_back(traj[1].getDerivative());

  start_time = msg->start_time;
  traj_id = msg->traj_id;
  traj_duration = traj[0].getTimeSum();
  receive_traj = true;

  ROS_INFO("[open_loop_controller] Receive traj %d, duration %.3fs.", traj_id, traj_duration);
}

/// @brief 定时按经过时间采样，起始前保持现状，结束后按配置保持终点；速度切向提供 yaw。
void publishOdom(const ros::TimerEvent&)
{
  const ros::Time now = ros::Time::now();

  if (!receive_traj)
  {
    publishState(now, current_pos, current_vel, last_yaw, 0.0);
    return;
  }

  const double elapsed = (now - start_time).toSec();

  if (elapsed < 0.0)
  {
    publishState(now, current_pos, current_vel, last_yaw, 0.0);
    return;
  }
  if (!hold_final_position && elapsed > traj_duration)
    return;

  const double t = std::min(std::max(elapsed, 0.0), traj_duration);
  Eigen::Vector3d pos = traj[0].evaluateDeBoorT(t);
  Eigen::Vector3d vel = Eigen::Vector3d::Zero();
  Eigen::Vector3d acc = Eigen::Vector3d::Zero();

  if (elapsed <= traj_duration)
  {
    vel = traj[1].evaluateDeBoorT(t);
    acc = traj[2].evaluateDeBoorT(t);
  }

  const double horizontal_speed = std::hypot(vel.x(), vel.y());
  if (horizontal_speed > yaw_min_speed)
    last_yaw = std::atan2(vel.y(), vel.x());

  const double yaw_rate = horizontal_speed > yaw_min_speed
                              ? (vel.x() * acc.y() - vel.y() * acc.x()) /
                                    std::max(horizontal_speed * horizontal_speed, 1e-6)
                              : 0.0;

  current_pos = pos;
  current_vel = vel;
  publishState(now, pos, vel, last_yaw, yaw_rate);
}
}  // namespace

/// @brief 读取初始状态和话题配置，订阅样条并以默认 100 Hz 发布直接采样得到的机身状态。
int main(int argc, char** argv)
{
  ros::init(argc, argv, "open_loop_controller");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string bspline_topic;
  std::string body_pose_topic;
  double publish_rate;
  double init_x;
  double init_y;
  double init_z;

  // 本段读取启动配置：下方“正常默认”来自当前 launch/XML，实际运行以参数服务器为准。
  // param() 的末项仅是缺参回退；getParam()/必需参数封装没有这一回退，具体见各行说明。
  // 参数读取：开环执行订阅的样条轨迹话题；收到后按消息 start_time 直接采样机身状态。
  // 配置位置：私有 /open_loop_controller/bspline_topic；当前主 launch 未覆盖此键，取下面的缺参回退
  // 。
  // 缺参回退："/planning/bspline"；只有参数服务器中没有该键时才使用。
  pnh.param<std::string>("bspline_topic", bspline_topic, "/planning/bspline");
  // 参数读取：机身 Odometry 接口；默认仿真 /quad_0/body_pose，真机由父 launch 切到
  // /LIO/odom_vehicle。
  // 配置来源：run.launch → /body_pose_topic；正常默认启动取 /quad_0/body_pose。
  // 缺参回退：std::string("/quad_0/body_pose")；只有参数服务器中没有该键时才使用。
  ros::param::param<std::string>("/body_pose_topic", body_pose_topic, std::string("/quad_0/body_pose"));
  // 参数读取：本节点输出消息所使用的世界参考系名称；名称设置本身不执行坐标变换。
  // 配置位置：私有 /open_loop_controller/frame_id；当前主 launch 未覆盖此键，取下面的缺参回退。
  // 缺参回退："world"；只有参数服务器中没有该键时才使用。
  pnh.param<std::string>("frame_id", frame_id, "world");
  // 参数读取：机身里程计的子坐标系名称，供消息使用者识别机身参考点。
  // 配置位置：私有 /open_loop_controller/child_frame_id；当前主 launch 未覆盖此键，取下面的缺参回退
  // 。
  // 缺参回退："quadruped"；只有参数服务器中没有该键时才使用。
  pnh.param<std::string>("child_frame_id", child_frame_id, "quadruped");
  // 参数读取：消息的目标发布频率（Hz），不等同于规划器重新生成轨迹的频率。
  // 配置位置：私有 /open_loop_controller/publish_rate；当前主 launch 未覆盖此键，取下面的缺参回退。
  // 缺参回退：100.0；只有参数服务器中没有该键时才使用。
  pnh.param("publish_rate", publish_rate, 100.0);
  // 参数读取：开环更新切向朝向的最低水平速度（m/s）；低于阈值保持上次朝向。
  // 配置位置：私有 /open_loop_controller/yaw_min_speed；当前主 launch 未覆盖此键，取下面的缺参回退
  // 。
  // 缺参回退：0.05；只有参数服务器中没有该键时才使用。
  pnh.param("yaw_min_speed", yaw_min_speed, 0.05);
  // 参数读取：开环轨迹结束后是否继续发布终点及零速度。
  // 配置位置：私有 /open_loop_controller/hold_final_position；当前主 launch 未覆盖此键，取下面的缺
  // 参回退。
  // 缺参回退：true；只有参数服务器中没有该键时才使用。
  pnh.param("hold_final_position", hold_final_position, true);
  // 参数读取：初始机身水平朝向（rad）；用于初始化仿真或开环采样朝向。
  // 配置位置：私有 /open_loop_controller/init_yaw；当前主 launch 未覆盖此键，取下面的缺参回退。
  // 缺参回退：0.0；只有参数服务器中没有该键时才使用。
  pnh.param("init_yaw", last_yaw, 0.0);
  // 参数读取：仿真机身初始世界坐标（m），由 simulator.xml 按导航模式设置，mode 1 为 (-19,1,0.25)。
  // 配置来源：simulator.xml → /init_x；正常默认启动取 -19.0。
  // 缺参回退：0.0；只有参数服务器中没有该键时才使用。
  ros::param::param("/init_x", init_x, 0.0);
  // 参数读取：仿真机身初始世界坐标（m），由 simulator.xml 按导航模式设置，mode 1 为 (-19,1,0.25)。
  // 配置来源：simulator.xml → /init_y；正常默认启动取 1.0。
  // 缺参回退：0.0；只有参数服务器中没有该键时才使用。
  ros::param::param("/init_y", init_y, 0.0);
  // 参数读取：仿真机身初始世界坐标（m），由 simulator.xml 按导航模式设置，mode 1 为 (-19,1,0.25)。
  // 配置来源：simulator.xml → /init_z；正常默认启动取 0.25。
  // 缺参回退：0.3；只有参数服务器中没有该键时才使用。
  ros::param::param("/init_z", init_z, 0.3);
  current_pos = Eigen::Vector3d(init_x, init_y, init_z);

  // 运行连接（输出入口）：机身 Odometry 接口，实际名称由 /body_pose_topic 指定；默认仿真
  // /quad_0/body_pose，接入规划/控制/感知/显示相应回调。
  // 队列容量：20 条消息，不是发布/规划频率。
  odom_pub = nh.advertise<nav_msgs::Odometry>(body_pose_topic, 20);
  ros::Subscriber bspline_sub =
      // 运行连接（接收入口）：开环样条输入，由私有 bspline_topic 指定，缺省 /planning/bspline；收到
      // 后按轨迹时间直接采样。
      // 队列容量：10 条消息，不是发布/规划频率。 到达后交给 bsplineCallback。
      nh.subscribe(bspline_topic, 10, bsplineCallback, ros::TransportHints().tcpNoDelay());

  // 定时任务：开环直接采样轨迹并发布机身状态，频率由 publish_rate 选择，缺省 100 Hz。
  ros::Timer timer = nh.createTimer(ros::Duration(1.0 / publish_rate), publishOdom);

  ROS_WARN("[open_loop_controller] ready. bspline: %s, odom: %s", bspline_topic.c_str(),
           body_pose_topic.c_str());

  ros::spin();
  return 0;
}
