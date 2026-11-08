/// @file
/// Go2 显示步态节点：根据机身水平速度生成 /joint_states，供 robot_state_publisher 驱动模型。
/// 关节相位/摆幅由经验正弦函数产生，不是逆动力学、落足规划或真实电机控制指令。
/// 默认 60 Hz 发布，速度过低或尚无位姿时使用站立姿态。
#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <vector>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

namespace {

constexpr double kPi = 3.14159265358979323846;

double clamp(double value, double low, double high) {
  return std::max(low, std::min(value, high));
}

/// @brief 封装里程计接收、站立/小跑姿态计算和 JointState 发布；关节顺序须与 URDF 名称对应。
class Go2GaitPublisher {
 public:
  /// @brief 加载步态显示参数、绑定 12 个关节名，并创建机身订阅和显示定时器。
  Go2GaitPublisher() : nh_(), pnh_("~") {
    // 本段读取启动配置：下方“正常默认”来自当前 launch/XML，实际运行以参数服务器为准。
    // param() 的末项仅是缺参回退；getParam()/必需参数封装没有这一回退，具体见各行说明。
    // 参数读取：关节动画消息的目标发布频率（Hz），不控制机身运动积分。
    // 配置位置：私有 /go2_gait_publisher/rate；当前主 launch 未覆盖此键，取下面的缺参回退。
    // 缺参回退：60.0；只有参数服务器中没有该键时才使用。
    pnh_.param("rate", rate_, 60.0);
    // 参数读取：关节动画的摆动频率（Hz），不是落足规划器或实机控制频率。
    // 配置位置：私有 /go2_gait_publisher/gait_frequency；当前主 launch 未覆盖此键，取下面的缺参回退
    // 。
    // 缺参回退：2.2；只有参数服务器中没有该键时才使用。
    pnh_.param("gait_frequency", gait_frequency_, 2.2);
    // 参数读取：动画速度映射的低速阈值（m/s），接近该值时趋于站立显示。
    // 配置位置：私有 /go2_gait_publisher/min_walk_speed；当前主 launch 未覆盖此键，取下面的缺参回退
    // 。
    // 缺参回退：0.05；只有参数服务器中没有该键时才使用。
    pnh_.param("min_walk_speed", min_walk_speed_, 0.05);
    // 参数读取：动画达到最大摆幅的速度尺度（m/s），不是机身控制限速。
    // 配置位置：私有 /go2_gait_publisher/max_walk_speed；当前主 launch 未覆盖此键，取下面的缺参回退
    // 。
    // 缺参回退：1.0；只有参数服务器中没有该键时才使用。
    pnh_.param("max_walk_speed", max_walk_speed_, 1.0);
    // 参数读取：是否持续使用固定速度比例的小跑动画。
    // 配置位置：私有 /go2_gait_publisher/always_trot；当前主 launch 未覆盖此键，取下面的缺参回退。
    // 缺参回退：false；只有参数服务器中没有该键时才使用。
    pnh_.param("always_trot", always_trot_, false);
    // 参数读取：髋关节动画摆幅尺度（rad），还会乘速度比例和相位系数。
    // 配置位置：私有 /go2_gait_publisher/hip_swing；当前主 launch 未覆盖此键，取下面的缺参回退。
    // 缺参回退：0.08；只有参数服务器中没有该键时才使用。
    pnh_.param("hip_swing", hip_swing_, 0.08);
    // 参数读取：大腿关节动画摆幅尺度（rad）。
    // 配置位置：私有 /go2_gait_publisher/thigh_swing；当前主 launch 未覆盖此键，取下面的缺参回退。
    // 缺参回退：0.32；只有参数服务器中没有该键时才使用。
    pnh_.param("thigh_swing", thigh_swing_, 0.32);
    // 参数读取：小腿关节动画摆幅尺度（rad）。
    // 配置位置：私有 /go2_gait_publisher/calf_swing；当前主 launch 未覆盖此键，取下面的缺参回退。
    // 缺参回退：0.42；只有参数服务器中没有该键时才使用。
    pnh_.param("calf_swing", calf_swing_, 0.42);

    std::string body_pose_topic;
    std::string joint_topic;
    // 参数读取：机身 Odometry 接口；默认仿真 /quad_0/body_pose，真机由父 launch 切到
    // /LIO/odom_vehicle。
    // 配置来源：run.launch → /body_pose_topic；正常默认启动取 /quad_0/body_pose。
    // 缺参回退：std::string("/quad_0/body_pose")；只有参数服务器中没有该键时才使用。
    ros::param::param<std::string>("/body_pose_topic", body_pose_topic, std::string("/quad_0/body_pose"));
    // 参数读取：关节动画的 JointState 输出，供 robot_state_publisher 更新模型 TF。
    // 配置位置：私有 /go2_gait_publisher/joint_topic；当前主 launch 未覆盖此键，取下面的缺参回退。
    // 缺参回退："/joint_states"；只有参数服务器中没有该键时才使用。
    pnh_.param<std::string>("joint_topic", joint_topic, "/joint_states");

    // 运行连接（接收入口）：机身 Odometry 接口，实际名称由 /body_pose_topic 指定；默认仿真
    // /quad_0/body_pose，接入规划/控制/感知/显示相应回调。
    // 队列容量：10 条消息，不是发布/规划频率。 到达后交给 odomCallback。
    odom_sub_ = nh_.subscribe(body_pose_topic, 10, &Go2GaitPublisher::odomCallback, this);
    // 运行连接（输出入口）：显示关节状态输出，私有 joint_topic 缺省 /joint_states，供
    // robot_state_publisher 消费。
    // 队列容量：10 条消息，不是发布/规划频率。
    joint_pub_ = nh_.advertise<sensor_msgs::JointState>(joint_topic, 10);
    // 定时任务：按 rate_ 发布显示关节角，缺省 60 Hz；不是四足动力学或实机电机控制循环。
    timer_ = nh_.createTimer(ros::Duration(1.0 / rate_), &Go2GaitPublisher::timerCallback, this);

    joint_msg_.name = {
        "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
        "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
        "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
        "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"};
    joint_msg_.position.resize(joint_msg_.name.size(), 0.0);
    joint_msg_.velocity.resize(joint_msg_.name.size(), 0.0);
  }

 private:
  /// @brief 优先使用里程计水平速度，速度较小时用相邻位置差估计速度，保持动画与机身运动一致。
  void odomCallback(const nav_msgs::OdometryConstPtr& odom) {
    const double vx = odom->twist.twist.linear.x;
    const double vy = odom->twist.twist.linear.y;
    const double twist_speed = std::sqrt(vx * vx + vy * vy);
    const ros::Time stamp = odom->header.stamp.isZero() ? ros::Time::now() : odom->header.stamp;
    const double x = odom->pose.pose.position.x;
    const double y = odom->pose.pose.position.y;

    double pose_speed = 0.0;
    if (has_prev_pose_) {
      const double dt = (stamp - last_odom_time_).toSec();
      if (dt > 1e-4) {
        const double dx = x - last_odom_x_;
        const double dy = y - last_odom_y_;
        pose_speed = std::sqrt(dx * dx + dy * dy) / dt;
      }
    }

    horizontal_speed_ = twist_speed > min_walk_speed_ ? twist_speed : pose_speed;
    last_odom_x_ = x;
    last_odom_y_ = y;
    last_odom_time_ = stamp;
    has_prev_pose_ = true;
    has_odom_ = true;
  }

  /// @brief 按速度映射摆幅并设置对角腿相位差，低速时回到站立状态。
  void timerCallback(const ros::TimerEvent& event) {
    if (!has_odom_) {
      publishStance(event.current_real);
      return;
    }

    const ros::Time stamp = event.current_real;

    const double speed_ratio =
        always_trot_ ? 0.45 : clamp((horizontal_speed_ - min_walk_speed_) /
                                        std::max(1e-3, max_walk_speed_ - min_walk_speed_),
                                    0.0, 1.0);

    if (speed_ratio <= 1e-3) {
      publishStance(stamp);
      return;
    }

    const double phase = 2.0 * kPi * gait_frequency_ * stamp.toSec();
    fillLeg(0, phase, speed_ratio, true);
    fillLeg(3, phase + kPi, speed_ratio, false);
    fillLeg(6, phase + kPi, speed_ratio, true);
    fillLeg(9, phase, speed_ratio, false);

    joint_msg_.header.stamp = stamp;
    joint_pub_.publish(joint_msg_);
  }

  /// @brief 填充固定站立关节角和零关节速度；角度单位 rad。
  void publishStance(const ros::Time& stamp) {
    const std::array<double, 12> stance = {
        0.05, 0.82, -1.58,
        -0.05, 0.82, -1.58,
        0.05, 0.95, -1.62,
        -0.05, 0.95, -1.62};

    for (size_t i = 0; i < stance.size(); ++i) {
      joint_msg_.position[i] = stance[i];
      joint_msg_.velocity[i] = 0.0;
    }

    joint_msg_.header.stamp = stamp;
    joint_pub_.publish(joint_msg_);
  }

  /// @brief 为一条腿生成髋/大腿/小腿的相位角并限幅，offset 指向该腿三个关节的起始索引。
  void fillLeg(size_t offset, double phase, double speed_ratio, bool left_side) {
    const double s = std::sin(phase);
    const double c = std::cos(phase);
    const double swing = std::max(0.0, s);
    const double side = left_side ? 1.0 : -1.0;

    const bool rear_leg = offset >= 6;
    const double thigh_stance = rear_leg ? 0.95 : 0.82;
    const double calf_stance = rear_leg ? -1.62 : -1.58;

    joint_msg_.position[offset + 0] =
        side * (0.05 + hip_swing_ * speed_ratio * 0.35 * s);
    joint_msg_.position[offset + 1] =
        thigh_stance + thigh_swing_ * speed_ratio * c;
    joint_msg_.position[offset + 2] =
        calf_stance + calf_swing_ * speed_ratio * swing - 0.10 * speed_ratio * (1.0 - swing);

    joint_msg_.position[offset + 0] = clamp(joint_msg_.position[offset + 0], -1.0, 1.0);
    joint_msg_.position[offset + 1] = clamp(joint_msg_.position[offset + 1], -1.2, 3.0);
    joint_msg_.position[offset + 2] = clamp(joint_msg_.position[offset + 2], -2.6, -0.9);

    const double omega = 2.0 * kPi * gait_frequency_;
    joint_msg_.velocity[offset + 0] = side * hip_swing_ * speed_ratio * 0.35 * c * omega;
    joint_msg_.velocity[offset + 1] = -thigh_swing_ * speed_ratio * s * omega;
    joint_msg_.velocity[offset + 2] = calf_swing_ * speed_ratio * (s > 0.0 ? c : 0.0) * omega;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber odom_sub_;
  ros::Publisher joint_pub_;
  ros::Timer timer_;

  // 12 个关节的显示角度/速度缓存，joint names 与 URDF 对应；不是发给真实电机的控制命令。
  sensor_msgs::JointState joint_msg_;
  // 上次机身观测时刻，用于从位置差估计水平速度。
  ros::Time last_odom_time_;

  // 当前动画使用的水平速度（m/s），优先用里程计速度，低速时改用位置差估计。
  double horizontal_speed_ = 0.0;
  // 上次机身世界 x（m），用于位置差速度计算。
  double last_odom_x_ = 0.0;
  // 上次机身世界 y（m）。
  double last_odom_y_ = 0.0;
  // 私有 rate 缺省 60 Hz，关节显示消息的目标发布频率。
  double rate_ = 60.0;
  // 私有 gait_frequency 缺省 2.2 Hz，用于动画相位，不是规划器重规划频率。
  double gait_frequency_ = 2.2;
  // 私有 min_walk_speed 缺省 0.05 m/s，低于此尺度时趋于站立动画。
  double min_walk_speed_ = 0.05;
  // 私有 max_walk_speed 缺省 1.0 m/s，用于把速度映射到最大动画摆幅。
  double max_walk_speed_ = 1.0;
  // 髋关节动画摆幅尺度，私有缺省 0.08 rad；实际摆幅还乘速度比例和相位因子。
  double hip_swing_ = 0.08;
  // 大腿动画摆幅尺度，私有缺省 0.32 rad。
  double thigh_swing_ = 0.32;
  // 小腿动画摆幅尺度，私有缺省 0.42 rad。
  double calf_swing_ = 0.42;
  // 私有 always_trot 缺省 false；true 时采用固定速度比例驱动小跑显示，即使实际速度低。
  bool always_trot_ = false;
  // 初始 false，收到观测后 true；未收到时只显示站立姿态。
  bool has_odom_ = false;
  // 初始 false，保存首帧后 true；只有已有前帧才计算位置差速度。
  bool has_prev_pose_ = false;
};

}  // namespace

/// @brief 创建动画对象并进入 ROS 回调循环；该节点本身不会发布机身运动命令。
int main(int argc, char** argv) {
  ros::init(argc, argv, "go2_gait_publisher");
  Go2GaitPublisher publisher;
  ros::spin();
  return 0;
}
