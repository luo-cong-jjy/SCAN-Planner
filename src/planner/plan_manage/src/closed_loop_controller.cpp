/// @file
/// 闭环轨迹跟踪节点：由 run.launch 的 closed_loop 分支启动。
/// 输入 /planning/bspline 与 /body_pose_topic 指向的 Odometry，输出机身系 /cmd_vel 和进度冻结 Bool。
/// 速度前馈与位置反馈在世界系计算，再转到机身 XY；z 不作为此平面控制器的执行自由度。
/// 参数由 advanced_param.xml 写入 /closed_loop_controller，控制周期目标为 0.01 s。
#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

#include <Eigen/Eigen>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <tf/tf.h>

#include "bspline_opt/uniform_bspline.h"
#include "scan_planner/Bspline.h"

namespace
{
using scan_planner::UniformBspline;

// 代码级转向限值 1.0 rad/s；读取私有 max_vyaw 后还会按此值封顶。
constexpr double kMaxVYawLimit = 1.0;

// 发布 /cmd_vel，供平面仿真或外部机身控制接口执行，不是关节电机命令。
ros::Publisher cmd_vel_pub;
// 发布 /planning/go2_execution_frozen；原地转向时为 true，通知 FSM 暂停轨迹时间。
ros::Publisher execution_frozen_pub;
// 接收 /planning/bspline 的最新可执行轨迹，触发 bsplineCallback()。
ros::Subscriber bspline_sub;
// 接收 /body_pose_topic 指向的机身状态，提供跟踪反馈；默认 /quad_0/body_pose。
ros::Subscriber odom_sub;
// 目标周期 0.01 s，调用 cmdCallback()，未收到必要输入时也持续输出停止指令。
ros::Timer cmd_timer;

// 初值 false；首次收到样条后 true，表示已有轨迹，不代表已到目标或数据始终新鲜。
bool receive_traj = false;
// 初值 false；机身回调后 true，允许使用 odom_pos/odom_yaw 计算反馈。
bool have_odom = false;
// 本控制器的轨迹缓存，接收时重建为 [位置,速度,加速度] 三条样条；控制主要取前两项。
std::vector<UniformBspline> traj;
// 当前样条时长（s），接收轨迹后从 getTimeSum() 得到，限制 exec_time 的上界。
double traj_duration = 0.0;
// 收到的轨迹编号，仅用于日志关联；不在此递增，来源是规划器消息。
int traj_id = 0;

// 实际世界位置（m），初值零、Odometry 回调更新；与 traj 的世界位置求水平误差。
Eigen::Vector3d odom_pos = Eigen::Vector3d::Zero();
// 实际机身水平朝向（rad），Odometry 四元数转换得到，用于转向误差和世界/机身速度变换。
double odom_yaw = 0.0;

// 控制器执行进度（s），每条新轨迹重置 0；原地转向暂停，正常跟踪按实际 dt 推进。
double exec_time = 0.0;
// 最近控制更新时间；用于计算下一步 dt，冻结时也刷新以避免恢复后的时间跳变。
ros::Time last_update_time;

// 私有 time_forward，默认配置 0.8 s；用前视位置估计朝向，不改变轨迹规划范围。
double time_forward;
// 私有 heading_error_threshold，默认配置 0.8 rad；超过时仅转向并冻结进度。
double heading_error_threshold;
// 私有 kp_pos，默认配置 0.8 1/s；世界系水平位置误差乘它后叠加到速度前馈。
double kp_pos;
// 私有 kp_yaw，默认配置 1.5 1/s；朝向误差乘它得到限幅前的角速度。
double kp_yaw;
// 私有 max_vx，默认配置 0.75 m/s；机身 x 速度分量的正负限幅。
double max_vx;
// 私有 max_vy，默认配置 0.35 m/s；机身 y 速度分量的正负限幅，合速度先按 max(max_vx,max_vy) 限制。
double max_vy;
// 私有 max_vyaw，默认配置并经代码封顶为 1.0 rad/s；限制转向指令。
double max_vyaw;
// 私有 finish_dist，默认配置 0.15 m；轨迹时间结束且水平位置误差小于它时发布全零速度。
double finish_dist;
// 从全局 /body_pose_topic 读取的反馈话题名；以上私有参数则由 advanced_param.xml 写入本节点命名空间。
std::string body_pose_topic;

/// @brief 从私有命名空间读取必需参数；缺失时打印具体键名，由 main() 停止启动，避免使用未初始化值。
bool loadRequiredParam(const ros::NodeHandle &nh, const std::string &name, double &value)
{
  // 参数读取：读取调用方要求的控制器私有参数；缺失时返回 false，由 loadParams() 汇总后使启动失败。
  // 具体参数名和作用见下方调用处。
  if (nh.getParam(name, value))
    return true;

  ROS_ERROR_STREAM("[closed_loop_controller] missing required private parameter ~" << name);
  return false;
}

/// @brief 加载跟踪增益、限幅和阈值；max_vyaw 还受到代码中 1 rad/s 上限约束。
bool loadParams(const ros::NodeHandle &nh)
{
  bool ok = true;
  // 本段读取启动配置：下方“正常默认”来自当前 launch/XML，实际运行以参数服务器为准。
  // param() 的末项仅是缺参回退；getParam()/必需参数封装没有这一回退，具体见各行说明。
  // 参数读取：机身 Odometry 接口；默认仿真 /quad_0/body_pose，真机由父 launch 切到
  // /LIO/odom_vehicle。
  // 配置来源：run.launch → /body_pose_topic；正常默认启动取 /quad_0/body_pose。
  // 缺参回退：std::string("/quad_0/body_pose")；只有参数服务器中没有该键时才使用。
  ros::param::param<std::string>("/body_pose_topic", body_pose_topic, std::string("/quad_0/body_pose"));
  // 参数读取：朝向前视时间，默认 0.8 s，用前视位置估计目标 yaw。
  // 配置来源：advanced_param.xml 的 /closed_loop_controller/time_forward；本接口无回退，缺参则启动
  // 失败。
  ok &= loadRequiredParam(nh, "time_forward", time_forward);
  // 参数读取：朝向误差阈值，默认 0.8 rad，超过时原地转向并冻结执行进度。
  // 配置来源：advanced_param.xml 的 /closed_loop_controller/heading_error_threshold；本接口无回退，
  // 缺参则启动失败。
  ok &= loadRequiredParam(nh, "heading_error_threshold", heading_error_threshold);
  // 参数读取：位置反馈增益，默认 0.8 1/s，水平位置误差转换为速度修正。
  // 配置来源：advanced_param.xml 的 /closed_loop_controller/kp_pos；本接口无回退，缺参则启动失败。
  ok &= loadRequiredParam(nh, "kp_pos", kp_pos);
  // 参数读取：朝向反馈增益，默认 1.5 1/s，角度误差转换为转向速度。
  // 配置来源：advanced_param.xml 的 /closed_loop_controller/kp_yaw；本接口无回退，缺参则启动失败。
  ok &= loadRequiredParam(nh, "kp_yaw", kp_yaw);
  // 参数读取：机身 x 速度限幅，默认 0.75 m/s。
  // 配置来源：advanced_param.xml 的 /closed_loop_controller/max_vx；本接口无回退，缺参则启动失败。
  ok &= loadRequiredParam(nh, "max_vx", max_vx);
  // 参数读取：机身 y 速度限幅，默认 0.35 m/s，水平合速度还会另行限幅。
  // 配置来源：advanced_param.xml 的 /closed_loop_controller/max_vy；本接口无回退，缺参则启动失败。
  ok &= loadRequiredParam(nh, "max_vy", max_vy);
  // 参数读取：转向速度限幅，默认 1.0 rad/s，代码还按 kMaxVYawLimit 封顶。
  // 配置来源：advanced_param.xml 的 /closed_loop_controller/max_vyaw；本接口无回退，缺参则启动失败
  // 。
  ok &= loadRequiredParam(nh, "max_vyaw", max_vyaw);
  // 参数读取：末端水平误差阈值，默认 0.15 m；时间结束且误差满足时才发布全零速度。
  // 配置来源：advanced_param.xml 的 /closed_loop_controller/finish_dist；本接口无回退，缺参则启动失
  // 败。
  ok &= loadRequiredParam(nh, "finish_dist", finish_dist);
  if (ok && max_vyaw > kMaxVYawLimit)
  {
    ROS_WARN("[closed_loop_controller] cap max_vyaw %.3f to %.3f rad/s.", max_vyaw, kMaxVYawLimit);
    max_vyaw = kMaxVYawLimit;
  }
  return ok;
}

/// @brief 将角度误差约束到 [-pi,pi]，避免跨越正负 pi 时走长方向转向。
double normalizeAngle(double angle)
{
  while (angle > M_PI)
    angle -= 2.0 * M_PI;
  while (angle < -M_PI)
    angle += 2.0 * M_PI;
  return angle;
}

/// @brief 按上下界裁剪标量，供机身各轴速度和角速度限幅。
double clamp(double value, double min_value, double max_value)
{
  return std::max(min_value, std::min(max_value, value));
}

/// @brief 限制二维合速度范数并保持原方向；机身 x/y 分量随后还会分别裁剪。
Eigen::Vector2d clampNorm(const Eigen::Vector2d &value, double max_norm)
{
  const double norm = value.norm();
  if (norm <= max_norm || norm < 1e-6)
    return value;
  return value / norm * max_norm;
}

/// @brief 用前视时间处的位置方向估计 yaw，退化时回退到切向速度，再退回当前机身朝向。
double estimateDesiredYaw(double t_cur, const Eigen::Vector3d &pos_des)
{
  const double t_look = std::min(traj_duration, t_cur + time_forward);
  Eigen::Vector3d dir = traj[0].evaluateDeBoorT(t_look) - pos_des;

  if (dir.head<2>().squaredNorm() < 1e-4)
  {
    Eigen::Vector3d vel = traj[1].evaluateDeBoorT(t_cur);
    dir = vel;
  }

  if (dir.head<2>().squaredNorm() < 1e-4)
    return odom_yaw;

  return std::atan2(dir(1), dir(0));
}

/// @brief 发布零平移速度，可保留受限角速度，因此既支持停车也支持原地转向。
void publishStop(double vyaw = 0.0)
{
  geometry_msgs::Twist cmd;
  cmd.angular.z = clamp(vyaw, -max_vyaw, max_vyaw);
  cmd_vel_pub.publish(cmd);
}

/// @brief 告知规划器是否暂停轨迹时间推进，避免原地转向期间参考继续向前移动。
void publishExecutionFrozen(bool frozen)
{
  std_msgs::Bool msg;
  msg.data = frozen;
  execution_frozen_pub.publish(msg);
}

/// @brief 把消息控制点/knots 重建为样条并求导，接收新轨迹时重置 exec_time。
/// 构造函数中的初始 interval 会被消息 knots 覆盖；本控制器不直接按消息 start_time 推进。
void bsplineCallback(const scan_planner::BsplineConstPtr &msg)
{
  Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());
  Eigen::VectorXd knots(msg->knots.size());

  for (size_t i = 0; i < msg->knots.size(); ++i)
    knots(i) = msg->knots[i];

  for (size_t i = 0; i < msg->pos_pts.size(); ++i)
  {
    pos_pts(0, i) = msg->pos_pts[i].x;
    pos_pts(1, i) = msg->pos_pts[i].y;
    pos_pts(2, i) = msg->pos_pts[i].z;
  }

  UniformBspline pos_traj(pos_pts, msg->order, 0.1);
  pos_traj.setKnot(knots);

  traj.clear();
  traj.push_back(pos_traj);
  traj.push_back(traj[0].getDerivative());
  traj.push_back(traj[1].getDerivative());

  traj_duration = traj[0].getTimeSum();
  traj_id = msg->traj_id;
  exec_time = 0.0;
  last_update_time = ros::Time::now();
  receive_traj = true;

  ROS_WARN("[closed_loop_controller] received bspline traj_id=%d duration=%.3f", traj_id, traj_duration);
}

/// @brief 更新实际世界位置和机身 yaw；跟踪误差需要与轨迹使用一致的世界坐标系。
void odomCallback(const nav_msgs::OdometryConstPtr &msg)
{
  odom_pos(0) = msg->pose.pose.position.x;
  odom_pos(1) = msg->pose.pose.position.y;
  odom_pos(2) = msg->pose.pose.position.z;
  odom_yaw = tf::getYaw(msg->pose.pose.orientation);
  have_odom = true;
}

/// @brief 控制定时入口：检查输入、估计朝向、必要时原地转向，否则计算前馈加比例反馈的水平速度。
/// 朝向误差过大时不增加 exec_time，转向完成后恢复轨迹时钟；末端按水平误差决定全零停车。
// 本层是机身轨迹跟踪：速度前馈+位置 P 反馈、yaw P 反馈与限幅；函数内没有预测模型和滚动最优控制求解。
void cmdCallback(const ros::TimerEvent &)
{
  if (!receive_traj || !have_odom)
  {
    publishExecutionFrozen(false);
    publishStop();
    return;
  }

  const ros::Time now = ros::Time::now();
  double dt = (now - last_update_time).toSec();
  if (dt < 0.0 || dt > 0.2)
    dt = 0.0;

  const double t_eval = std::min(exec_time, traj_duration);
  Eigen::Vector3d pos_des = traj[0].evaluateDeBoorT(t_eval);
  Eigen::Vector3d vel_des = traj[1].evaluateDeBoorT(t_eval);

  const double yaw_des = estimateDesiredYaw(t_eval, pos_des);
  const double yaw_err = normalizeAngle(yaw_des - odom_yaw);
  // yaw 比例反馈：目标与实际朝向之差（rad）乘增益，得到角速度（rad/s），再按最大角速度裁剪。
  const double vyaw_cmd = clamp(kp_yaw * yaw_err, -max_vyaw, max_vyaw);

  // 大朝向误差下先旋转，刷新 last_update_time 以防恢复时将整段等待时间计入轨迹进度。
  if (std::abs(yaw_err) > heading_error_threshold)
  {
    publishExecutionFrozen(true);
    publishStop(vyaw_cmd);
    last_update_time = now; // freeze exec_time while rotating in place
    return;
  }

  publishExecutionFrozen(false);
  exec_time = std::min(traj_duration, exec_time + dt);
  last_update_time = now;

  pos_des = traj[0].evaluateDeBoorT(exec_time);
  vel_des = traj[1].evaluateDeBoorT(exec_time);

  // 同一世界系下的水平跟踪误差 e_p=p_d-p_actual；vel_ff 为已有轨迹给出的期望水平速度。
  Eigen::Vector2d pos_err(pos_des(0) - odom_pos(0), pos_des(1) - odom_pos(1));
  Eigen::Vector2d vel_ff(vel_des(0), vel_des(1));
  // 世界系速度前馈叠加位置误差反馈后先做合速度限幅；kp_pos 单位为 1/s。
  // 未限幅时 v_cmd=v_d+kp_pos*e_p：落后于计划位置时给出追赶修正；这里没有误差积分项或误差微分项。
  Eigen::Vector2d vel_world = clampNorm(vel_ff + kp_pos * pos_err, std::max(max_vx, max_vy));

  const double c = std::cos(odom_yaw);
  const double s = std::sin(odom_yaw);
  geometry_msgs::Twist cmd;
  // R(yaw) 的转置将世界水平速度转换到机身系，再逐轴限幅；角速度另由 yaw 比例控制得到。
  cmd.linear.x = clamp(c * vel_world(0) + s * vel_world(1), -max_vx, max_vx);
  cmd.linear.y = clamp(-s * vel_world(0) + c * vel_world(1), -max_vy, max_vy);
  cmd.angular.z = vyaw_cmd;

  if (exec_time >= traj_duration && pos_err.norm() < finish_dist)
    cmd = geometry_msgs::Twist();

  // 发布的是机身前进/侧移速度和 yaw 角速度；真机需外部运动接口执行，当前仿真由 go2_kinematic_sim 积分。
  // 此消息与 go2_gait_publisher 的显示动画均不提供电机力矩/接触动力学控制实现。
  cmd_vel_pub.publish(cmd);
}
} // namespace

/// @brief 注册轨迹、实际位姿和控制定时器；无有效轨迹/位姿时保持发布停止指令。
int main(int argc, char **argv)
{
  ros::init(argc, argv, "closed_loop_controller");
  ros::NodeHandle node;
  ros::NodeHandle nh("~");

  if (!loadParams(nh))
    return 1;

  // 运行连接（接收入口）：默认解析为 /planning/bspline，执行控制器接收 FSM 的可执行轨迹。
  // 队列容量：10 条消息，不是发布/规划频率。 到达后交给 bsplineCallback。
  bspline_sub = node.subscribe("planning/bspline", 10, bsplineCallback);
  // 运行连接（接收入口）：机身 Odometry 接口，实际名称由 /body_pose_topic 指定；默认仿真
  // /quad_0/body_pose，接入规划/控制/感知/显示相应回调。
  // 队列容量：20 条消息，不是发布/规划频率。 到达后交给 odomCallback。
  odom_sub = node.subscribe(body_pose_topic, 20, odomCallback, ros::TransportHints().tcpNoDelay());
  // 运行连接（输出入口）：默认解析为 /cmd_vel，闭环控制器给出的机身 x/y 速度和 z 轴转向角速度，供仿
  // 真或外部机身接口执行。
  // 队列容量：20 条消息，不是发布/规划频率。
  cmd_vel_pub = node.advertise<geometry_msgs::Twist>("cmd_vel", 20);
  // 运行连接（输出入口）：默认解析为 /planning/go2_execution_frozen，通知 FSM 是否暂停轨迹执行进度
  // 。
  // 队列容量：10 条消息，不是发布/规划频率。
  execution_frozen_pub = node.advertise<std_msgs::Bool>("planning/go2_execution_frozen", 10);
  // 定时任务：闭环控制，周期 0.01 s；计算机身速度，朝向误差过大时只转向并冻结进度。
  cmd_timer = node.createTimer(ros::Duration(0.01), cmdCallback);

  last_update_time = ros::Time::now();
  ROS_WARN("[closed_loop_controller] ready.");

  ros::spin();
  return 0;
}
