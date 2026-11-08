/// @file
/// 当前默认复现的运动学执行模型：按速度数值积分机身状态，不求解刚体受力、足地接触或障碍碰撞反作用。
/// 平面机身运动学仿真，closed_loop 且非真机模式时由 run.launch 启动。
/// 接收机身系 Twist，按当前 yaw 转到世界系并积分，向各模块提供 /quad_0/body_pose。
/// 仅积分 x/y/yaw，z 固定；不模拟接触、关节动力学、地形支撑或真实四足控制器。
#include <algorithm>
#include <cmath>
#include <string>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

namespace
{
constexpr double kMaxVYawLimit = 1.0;

// 发布当前积分后的机身 Odometry，默认 /quad_0/body_pose，供规划、控制、感知和显示共用。
ros::Publisher odom_pub;
// 订阅 /cmd_vel，缓存闭环控制器给出的机身速度。
ros::Subscriber cmd_sub;
// 按 sim_rate 触发积分与里程计发布；实际 dt 由 ROS 时钟差计算。
ros::Timer sim_timer;
// 可选机身 TF 发布入口；默认 publish_tf=false，world→base 由 odom_visualization 发布。
tf::TransformBroadcaster *tf_broadcaster = nullptr;

// 机身世界 x（m）；main() 从 /init_x 设置，默认 mode 1 为 -19，随后按世界系速度积分。
double x = 0.0;
// 机身世界 y（m）；来自 /init_y，默认 mode 1 为 1，随后积分更新。
double y = 0.0;
// 机身世界 z（m）；来自 /init_z，默认 mode 1 为 0.25，此平面仿真不积分 z。
double z = 0.0;
// 机身朝向（rad），私有 init_yaw 缺省 0；根据角速度积分并归一化。
double yaw = 0.0;

// 最近接收并限幅的机身 x 速度（m/s），初值 0。
double vx_cmd = 0.0;
// 最近接收并限幅的机身 y 速度（m/s），初值 0。
double vy_cmd = 0.0;
// 最近接收并限幅的转向角速度（rad/s），初值 0；积分时另检查指令超时。
double vyaw_cmd = 0.0;
// 当前用于积分的世界 x 速度（m/s），由机身速度结合 yaw 旋转得到，随 Odometry 输出。
double vx_world = 0.0;
// 当前用于积分的世界 y 速度（m/s）。
double vy_world = 0.0;

// 声明初值 0.8；main() 优先读私有 max_vx，再读 /closed_loop_controller/max_vx，正常默认配置实际为 0.75 m/s。
double max_vx = 0.8;
// 声明初值 0.5；同样按私有/控制器配置覆盖，正常默认配置实际为 0.35 m/s。
double max_vy = 0.5;
// 转向速度限幅（rad/s），正常配置 1.0，并受代码级 1.0 上限约束。
double max_vyaw = kMaxVYawLimit;
// 私有 cmd_timeout 缺省 0.3 s；距最近指令超时后，本次积分按零速度处理。
double cmd_timeout = 0.3;
// 私有 sim_rate 缺省 100 Hz，只规定目标触发频率，dt 异常时积分步长会置零。
double sim_rate = 100.0;
// 私有 publish_tf 缺省 false，避免默认链路重复广播机身 TF。
bool publish_tf = false;
// 机身输出的世界参考系名，私有 frame_id 缺省 world。
std::string frame_id = "world";
// 机身输出的子坐标系名，私有 child_frame_id 缺省 base。
std::string child_frame_id = "base";
// 读取全局 /body_pose_topic 选择输出话题；声明默认 /quad_0/body_pose。
std::string body_pose_topic = "/quad_0/body_pose";

// 最近指令接收时刻，main() 初始化当前时间，用于超时保护。
ros::Time last_cmd_time;
// 上次积分时刻，main() 初始化当前时间；每次仿真回调刷新并计算 dt。
ros::Time last_sim_time;

/// @brief 限制输入机身速度或角速度，防止仿真积分使用超出配置的指令。
double clamp(double value, double min_value, double max_value)
{
  return std::max(min_value, std::min(max_value, value));
}

/// @brief 将积分后的 yaw 归一化到 [-pi,pi]，单位 rad。
double normalizeAngle(double angle)
{
  while (angle > M_PI)
    angle -= 2.0 * M_PI;
  while (angle < -M_PI)
    angle += 2.0 * M_PI;
  return angle;
}

/// @brief 优先读取本节点私有参数，缺失时复用全局控制器参数，最后使用代码默认值。
void loadParamWithFallback(const ros::NodeHandle &nh, const std::string &private_name,
                           const std::string &fallback_name, double &value, double default_value)
{
  // 参数读取：优先读取本仿真节点私有覆盖项；缺失后继续尝试下方指定的控制器配置，再使用调用方给出的
  // 回退值。
  if (nh.getParam(private_name, value))
    return;
  // 私有覆盖不存在时，读取调用方指定的控制器参数；再失败才使用 default_value。
  if (ros::param::get(fallback_name, value))
    return;
  value = default_value;
}

/// @brief 缓存限幅后的机身速度并记录接收时间；运动推进由 simCallback() 统一完成。
void cmdCallback(const geometry_msgs::TwistConstPtr &msg)
{
  vx_cmd = clamp(msg->linear.x, -max_vx, max_vx);
  vy_cmd = clamp(msg->linear.y, -max_vy, max_vy);
  vyaw_cmd = clamp(msg->angular.z, -max_vyaw, max_vyaw);
  last_cmd_time = ros::Time::now();
}

/// @brief 发布世界位置、朝向、世界系水平速度和角速度；可选 TF 默认关闭以避免重复发布 world→base。
void publishOdom(const ros::Time &stamp)
{
  geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(yaw);

  nav_msgs::Odometry odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = frame_id;
  odom.child_frame_id = child_frame_id;
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = z;
  odom.pose.pose.orientation = q;
  odom.twist.twist.linear.x = vx_world;
  odom.twist.twist.linear.y = vy_world;
  odom.twist.twist.angular.z = vyaw_cmd;
  odom_pub.publish(odom);

  if (!publish_tf || tf_broadcaster == nullptr)
    return;

  geometry_msgs::TransformStamped tf_msg;
  tf_msg.header.stamp = stamp;
  tf_msg.header.frame_id = frame_id;
  tf_msg.child_frame_id = child_frame_id;
  tf_msg.transform.translation.x = x;
  tf_msg.transform.translation.y = y;
  tf_msg.transform.translation.z = z;
  tf_msg.transform.rotation = q;
  tf_broadcaster->sendTransform(tf_msg);
}

/// @brief 按真实回调时间差积分世界系 XY 和 yaw；异常时间步置零，指令超时则按零速度积分。
void simCallback(const ros::TimerEvent &)
{
  const ros::Time now = ros::Time::now();
  double dt = (now - last_sim_time).toSec();
  last_sim_time = now;
  if (dt < 0.0 || dt > 0.2)
    dt = 0.0;

  double vx = vx_cmd;
  double vy = vy_cmd;
  double wz = vyaw_cmd;
  // 通信停止后由仿真侧超时保护清除本轮速度，区别于控制器主动发布零指令。
  if ((now - last_cmd_time).toSec() > cmd_timeout)
  {
    vx = 0.0;
    vy = 0.0;
    wz = 0.0;
  }

  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  vx_world = c * vx - s * vy;
  vy_world = s * vx + c * vy;

  x += vx_world * dt;
  y += vy_world * dt;
  yaw = normalizeAngle(yaw + wz * dt);

  publishOdom(now);
}
} // namespace

/// @brief 读取初始位置、速度上限和发布频率，注册速度输入及目标周期 100 Hz 的仿真定时器。
int main(int argc, char **argv)
{
  ros::init(argc, argv, "go2_kinematic_sim");
  ros::NodeHandle node;
  ros::NodeHandle nh("~");

  // 本段读取启动配置：下方“正常默认”来自当前 launch/XML，实际运行以参数服务器为准。
  // param() 的末项仅是缺参回退；getParam()/必需参数封装没有这一回退，具体见各行说明。
  // 参数读取：机身 Odometry 接口；默认仿真 /quad_0/body_pose，真机由父 launch 切到
  // /LIO/odom_vehicle。
  // 配置来源：run.launch → /body_pose_topic；正常默认启动取 /quad_0/body_pose。
  // 缺参回退：std::string("/quad_0/body_pose")；只有参数服务器中没有该键时才使用。
  ros::param::param("/body_pose_topic", body_pose_topic, std::string("/quad_0/body_pose"));
  // 参数读取：仿真机身初始世界坐标（m），由 simulator.xml 按导航模式设置，mode 1 为 (-19,1,0.25)。
  // 配置来源：simulator.xml → /init_x；正常默认启动取 -19.0。
  // 缺参回退：0.0；只有参数服务器中没有该键时才使用。
  ros::param::param("/init_x", x, 0.0);
  // 参数读取：仿真机身初始世界坐标（m），由 simulator.xml 按导航模式设置，mode 1 为 (-19,1,0.25)。
  // 配置来源：simulator.xml → /init_y；正常默认启动取 1.0。
  // 缺参回退：0.0；只有参数服务器中没有该键时才使用。
  ros::param::param("/init_y", y, 0.0);
  // 参数读取：仿真机身初始世界坐标（m），由 simulator.xml 按导航模式设置，mode 1 为 (-19,1,0.25)。
  // 配置来源：simulator.xml → /init_z；正常默认启动取 0.25。
  // 缺参回退：0.3；只有参数服务器中没有该键时才使用。
  ros::param::param("/init_z", z, 0.3);
  // 参数读取：初始机身水平朝向（rad）；用于初始化仿真或开环采样朝向。
  // 配置位置：私有 /go2_kinematic_sim/init_yaw；当前主 launch 未覆盖此键，取下面的缺参回退。
  // 缺参回退：0.0；只有参数服务器中没有该键时才使用。
  nh.param("init_yaw", yaw, 0.0);
  // 参数读取：x 速度限幅，正常默认 0.75 m/s，最终回退 0.8。
  // 查找顺序：本仿真节点私有覆盖 → /closed_loop_controller/max_vx → 当前调用给出的回退值。
  loadParamWithFallback(nh, "max_vx", "/closed_loop_controller/max_vx", max_vx, 0.8);
  // 参数读取：y 速度限幅，正常默认 0.35 m/s，最终回退 0.5。
  // 查找顺序：本仿真节点私有覆盖 → /closed_loop_controller/max_vy → 当前调用给出的回退值。
  loadParamWithFallback(nh, "max_vy", "/closed_loop_controller/max_vy", max_vy, 0.5);
  // 参数读取：转向速度限幅，正常默认 1.0 rad/s，最终回退代码级上限。
  // 查找顺序：本仿真节点私有覆盖 → /closed_loop_controller/max_vyaw → 当前调用给出的回退值。
  loadParamWithFallback(nh, "max_vyaw", "/closed_loop_controller/max_vyaw", max_vyaw, kMaxVYawLimit);
  if (max_vyaw > kMaxVYawLimit)
  {
    ROS_WARN("[Go2 kinematic sim] cap max_vyaw %.3f to %.3f rad/s.", max_vyaw, kMaxVYawLimit);
    max_vyaw = kMaxVYawLimit;
  }
  // 参数读取：速度指令超时阈值（s）；仿真在超时后按零速度积分。
  // 配置位置：私有 /go2_kinematic_sim/cmd_timeout；当前主 launch 未覆盖此键，取下面的缺参回退。
  // 缺参回退：0.3；只有参数服务器中没有该键时才使用。
  nh.param("cmd_timeout", cmd_timeout, 0.3);
  // 参数读取：平面运动学积分及里程计发布的目标频率（Hz）。
  // 配置位置：私有 /go2_kinematic_sim/sim_rate；当前主 launch 未覆盖此键，取下面的缺参回退。
  // 缺参回退：100.0；只有参数服务器中没有该键时才使用。
  nh.param("sim_rate", sim_rate, 100.0);
  // 参数读取：是否由机身仿真广播机身 TF；默认关闭，避免与 odom_visualization 重复。
  // 配置位置：私有 /go2_kinematic_sim/publish_tf；当前主 launch 未覆盖此键，取下面的缺参回退。
  // 缺参回退：false；只有参数服务器中没有该键时才使用。
  nh.param("publish_tf", publish_tf, false);
  // 参数读取：本节点输出消息所使用的世界参考系名称；名称设置本身不执行坐标变换。
  // 配置位置：私有 /go2_kinematic_sim/frame_id；当前主 launch 未覆盖此键，取下面的缺参回退。
  // 缺参回退：std::string("world")；只有参数服务器中没有该键时才使用。
  nh.param("frame_id", frame_id, std::string("world"));
  // 参数读取：机身里程计的子坐标系名称，供消息使用者识别机身参考点。
  // 配置位置：私有 /go2_kinematic_sim/child_frame_id；当前主 launch 未覆盖此键，取下面的缺参回退。
  // 缺参回退：std::string("base")；只有参数服务器中没有该键时才使用。
  nh.param("child_frame_id", child_frame_id, std::string("base"));

  tf::TransformBroadcaster broadcaster;
  tf_broadcaster = &broadcaster;

  // 运行连接（输出入口）：机身 Odometry 接口，实际名称由 /body_pose_topic 指定；默认仿真
  // /quad_0/body_pose，接入规划/控制/感知/显示相应回调。
  // 队列容量：100 条消息，不是发布/规划频率。
  odom_pub = node.advertise<nav_msgs::Odometry>(body_pose_topic, 100);
  // 运行连接（接收入口）：默认解析为 /cmd_vel，闭环控制器给出的机身 x/y 速度和 z 轴转向角速度，供仿
  // 真或外部机身接口执行。
  // 队列容量：20 条消息，不是发布/规划频率。 到达后交给 cmdCallback。
  cmd_sub = node.subscribe("cmd_vel", 20, cmdCallback, ros::TransportHints().tcpNoDelay());

  last_cmd_time = ros::Time::now();
  last_sim_time = ros::Time::now();
  // 定时任务：按 sim_rate 积分机身 x/y/yaw，默认 100 Hz；实际积分使用相邻回调的时钟差。
  sim_timer = node.createTimer(ros::Duration(1.0 / sim_rate), simCallback);

  ROS_WARN("[Go2 kinematic sim] ready.");

  ros::spin();
  return 0;
}
