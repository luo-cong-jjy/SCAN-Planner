/// @file
/// 备用单目标到路径消息转换工具，默认 run.launch 没有启动它。
/// 接收 RViz 目标和机身位姿，输出私有 ~waypoints 与 ~waypoints_vis；当前 FSM mode 1 直接订阅目标。
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include <string>

ros::Publisher waypoints_pub;
ros::Publisher waypoints_vis_pub;
nav_msgs::Odometry odom;
bool is_odom_ready = false;

/// @brief 缓存最近机身位姿，仅用于生成可视化起点。
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  is_odom_ready = true;
  odom = *msg;
}

/// @brief 将目标封装为 nav_msgs/Path，并另用 PoseArray 显示起终点。
void publishWaypoints(const geometry_msgs::PoseStamped& goal) {
  nav_msgs::Path waypoints;
  waypoints.header.frame_id = "world";
  waypoints.header.stamp = ros::Time::now();
  waypoints.poses.push_back(goal);
  waypoints_pub.publish(waypoints);

  geometry_msgs::PoseArray waypoints_vis;
  waypoints_vis.header = waypoints.header;
  if (is_odom_ready) {
    waypoints_vis.poses.push_back(odom.pose.pose);
  }
  waypoints_vis.poses.push_back(goal.pose);
  waypoints_vis_pub.publish(waypoints_vis);
}

/// @brief 拒绝高度小于等于 -0.1 的目标，其余目标转发给路径封装函数。
void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  if (msg->pose.position.z <= -0.1) {
    ROS_WARN("[waypoint_generator] invalid goal.");
    return;
  }

  publishWaypoints(*msg);
}

/// @brief 注册全局目标输入和位姿订阅，以及当前节点私有路径发布者。
int main(int argc, char** argv) {
  ros::init(argc, argv, "waypoint_generator");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string body_pose_topic;
  // 本段读取启动配置：下方“正常默认”来自当前 launch/XML，实际运行以参数服务器为准。
  // param() 的末项仅是缺参回退；getParam()/必需参数封装没有这一回退，具体见各行说明。
  // 参数读取：机身 Odometry 接口；默认仿真 /quad_0/body_pose，真机由父 launch 切到
  // /LIO/odom_vehicle。
  // 配置来源：run.launch → /body_pose_topic；正常默认启动取 /quad_0/body_pose。
  // 缺参回退：std::string("/quad_0/body_pose")；只有参数服务器中没有该键时才使用。
  ros::param::param<std::string>("/body_pose_topic", body_pose_topic, std::string("/quad_0/body_pose"));

  // 运行连接（接收入口）：机身 Odometry 接口，实际名称由 /body_pose_topic 指定；默认仿真
  // /quad_0/body_pose，接入规划/控制/感知/显示相应回调。
  // 队列容量：10 条消息，不是发布/规划频率。 到达后交给 odomCallback。
  ros::Subscriber odom_sub = nh.subscribe(body_pose_topic, 10, odomCallback);
  // 运行连接（接收入口）：RViz 手动单目标入口（PoseStamped）；mode 1 的 FSM 据此建立导航任务。
  // 队列容量：10 条消息，不是发布/规划频率。 到达后交给 goalCallback。
  ros::Subscriber goal_sub = nh.subscribe("/move_base_simple/goal", 10, goalCallback);
  // 运行连接（输出入口）：备用 waypoint_generator 的私有 Path 输出，默认主 FSM 并不订阅这一路。
  // 队列容量：50 条消息，不是发布/规划频率。
  waypoints_pub = pnh.advertise<nav_msgs::Path>("waypoints", 50);
  // 运行连接（输出入口）：备用 waypoint_generator 的私有目标序列显示（PoseArray）。
  // 队列容量：10 条消息，不是发布/规划频率。
  waypoints_vis_pub = pnh.advertise<geometry_msgs::PoseArray>("waypoints_vis", 10);

  ros::spin();
  return 0;
}
