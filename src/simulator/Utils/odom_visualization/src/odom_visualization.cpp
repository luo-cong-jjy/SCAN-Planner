/// @file
/// 里程计可视化节点：由 simulator.xml 启动，在仿真和真机模式均可接收机身 Odometry。
/// 输出私有 pose/path/velocity 等话题和模型显示，并按配置发布 world→base 及原始传感器显示 TF。
/// sliding_map 的 TF 由 GridMap 发布，不能把所有显示坐标系都归到本节点。
#include <iostream>
#include <string>
#include <string.h>
#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include "sensor_msgs/Range.h"
#include "visualization_msgs/Marker.h"
#include "armadillo"
#include "pose_utils.h"

using namespace arma;
using namespace std;

static  string mesh_resource;
static double color_r, color_g, color_b, color_a, cov_scale, scale;

bool   cross_config = false;
bool   tf45       = false;
bool   cov_pos    = false;
bool   cov_vel    = false;
bool   cov_color  = false;
bool   origin       = false;
bool   isOriginSet  = false;
colvec poseOrigin(6);
ros::Publisher posePub;
ros::Publisher pathPub;
ros::Publisher velPub;
ros::Publisher covPub;
ros::Publisher covVelPub;
ros::Publisher trajPub;
ros::Publisher sensorPub;
ros::Publisher meshPub;
ros::Publisher heightPub;
tf::TransformBroadcaster* broadcaster;
geometry_msgs::PoseStamped poseROS;
nav_msgs::Path             pathROS;
visualization_msgs::Marker velROS;
visualization_msgs::Marker covROS;
visualization_msgs::Marker covVelROS;
visualization_msgs::Marker trajROS;
visualization_msgs::Marker sensorROS;
visualization_msgs::Marker meshROS;
sensor_msgs::Range         heightROS;
string _frame_id;

/// @brief 将机身位置、姿态和速度转换为历史路径、Marker、模型姿态及 TF；不参与轨迹控制反馈计算。
void odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  if (msg->header.frame_id == string("null"))
    return;
  colvec pose(6);  
  pose(0) = msg->pose.pose.position.x;
  pose(1) = msg->pose.pose.position.y;
  pose(2) = msg->pose.pose.position.z;
  colvec q(4);
  q(0)    = msg->pose.pose.orientation.w;
  q(1)    = msg->pose.pose.orientation.x;
  q(2)    = msg->pose.pose.orientation.y;
  q(3)    = msg->pose.pose.orientation.z;
  pose.rows(3,5) = R_to_ypr(quaternion_to_R(q));
  colvec vel(3);
  vel(0) = msg->twist.twist.linear.x;
  vel(1) = msg->twist.twist.linear.y;
  vel(2) = msg->twist.twist.linear.z;  
  
  if (origin && !isOriginSet)
  {
    isOriginSet = true;
    poseOrigin  = pose;
  }
  if (origin)
  {
    vel  = trans(ypr_to_R(pose.rows(3,5))) * vel;  
    pose = pose_update(pose_inverse(poseOrigin), pose);
    vel  = ypr_to_R(pose.rows(3,5)) * vel;  
  }

  // Pose
  poseROS.header = msg->header;
  poseROS.header.stamp = msg->header.stamp;
  poseROS.header.frame_id = string("world");
  poseROS.pose.position.x = pose(0);
  poseROS.pose.position.y = pose(1);
  poseROS.pose.position.z = pose(2);
  q = R_to_quaternion(ypr_to_R(pose.rows(3,5)));
  poseROS.pose.orientation.w = q(0);
  poseROS.pose.orientation.x = q(1);
  poseROS.pose.orientation.y = q(2);
  poseROS.pose.orientation.z = q(3);      
  posePub.publish(poseROS);
  
  // Velocity
  colvec yprVel(3);
  yprVel(0) =  atan2(vel(1), vel(0));
  yprVel(1) = -atan2(vel(2), norm(vel.rows(0,1),2));
  yprVel(2) = 0;
  q = R_to_quaternion(ypr_to_R(yprVel));    
  velROS.header.frame_id = string("world");
  velROS.header.stamp = msg->header.stamp;
  velROS.ns = string("velocity");
  velROS.id = 0;
  velROS.type = visualization_msgs::Marker::ARROW;
  velROS.action = visualization_msgs::Marker::ADD;
  velROS.pose.position.x = pose(0);
  velROS.pose.position.y = pose(1);
  velROS.pose.position.z = pose(2);
  velROS.pose.orientation.w = q(0);
  velROS.pose.orientation.x = q(1);
  velROS.pose.orientation.y = q(2);
  velROS.pose.orientation.z = q(3);      
  velROS.scale.x = norm(vel, 2); 
  velROS.scale.y = 0.05;
  velROS.scale.z = 0.05;
  velROS.color.a = 1.0;
  velROS.color.r = color_r;
  velROS.color.g = color_g;
  velROS.color.b = color_b;
  velPub.publish(velROS);

  // Path
  static bool has_last_path_pose = false;
  static geometry_msgs::Point last_path_position;
  const geometry_msgs::Point& curr_path_position = poseROS.pose.position;
  const double dx = curr_path_position.x - last_path_position.x;
  const double dy = curr_path_position.y - last_path_position.y;
  const double dz = curr_path_position.z - last_path_position.z;
  if (!has_last_path_pose || dx * dx + dy * dy + dz * dz > 0.25)
  {
    pathROS.header = poseROS.header;
    pathROS.poses.push_back(poseROS);
    last_path_position = curr_path_position;
    has_last_path_pose = true;
    while (pathROS.poses.size() > 1000)
      pathROS.poses.erase(pathROS.poses.begin());
    pathPub.publish(pathROS);
  }

  // Covariance color
  double r = 1;
  double g = 1;
  double b = 1;
  bool G = msg->twist.covariance[33];
  bool V = msg->twist.covariance[34];
  bool L = msg->twist.covariance[35];
  if (cov_color)
  {
    r = G;
    g = V;
    b = L;
  }
  
  // Covariance Position
  if (cov_pos)
  {
    mat P(6,6);
    for (int j = 0; j < 6; j++)
      for (int i = 0; i < 6; i++)
        P(i,j) = msg->pose.covariance[i+j*6];
    colvec eigVal;
    mat    eigVec;
    eig_sym(eigVal, eigVec, P.submat(0,0,2,2));
    if (det(eigVec) < 0)
    {
      for (int k = 0; k < 3; k++)
      {
        mat eigVecRev = eigVec;
        eigVecRev.col(k) *= -1;
        if (det(eigVecRev) > 0)
        {
          eigVec = eigVecRev;
          break;
        }
      }
    }
    covROS.header.frame_id = string("world");
    covROS.header.stamp = msg->header.stamp;
    covROS.ns = string("covariance");
    covROS.id = 0;
    covROS.type = visualization_msgs::Marker::SPHERE;
    covROS.action = visualization_msgs::Marker::ADD;
    covROS.pose.position.x = pose(0);
    covROS.pose.position.y = pose(1);
    covROS.pose.position.z = pose(2);
    q = R_to_quaternion(eigVec);
    covROS.pose.orientation.w = q(0);
    covROS.pose.orientation.x = q(1);
    covROS.pose.orientation.y = q(2);
    covROS.pose.orientation.z = q(3);      
    covROS.scale.x = sqrt(eigVal(0))*cov_scale;
    covROS.scale.y = sqrt(eigVal(1))*cov_scale;
    covROS.scale.z = sqrt(eigVal(2))*cov_scale;
    covROS.color.a = 0.4;
    covROS.color.r = r * 0.5;
    covROS.color.g = g * 0.5;
    covROS.color.b = b * 0.5;
    covPub.publish(covROS);
  }

  // Covariance Velocity
  if (cov_vel)
  {
    mat P(3,3);
    for (int j = 0; j < 3; j++)
      for (int i = 0; i < 3; i++)
        P(i,j) = msg->twist.covariance[i+j*6];
    mat R = ypr_to_R(pose.rows(3,5));
    P = R * P * trans(R);
    colvec eigVal;
    mat    eigVec;
    eig_sym(eigVal, eigVec, P);
    if (det(eigVec) < 0)
    {
      for (int k = 0; k < 3; k++)
      {
        mat eigVecRev = eigVec;
        eigVecRev.col(k) *= -1;
        if (det(eigVecRev) > 0)
        {
          eigVec = eigVecRev;
          break;
        }
      }
    }
    covVelROS.header.frame_id = string("world");
    covVelROS.header.stamp = msg->header.stamp;
    covVelROS.ns = string("covariance_velocity");
    covVelROS.id = 0;
    covVelROS.type = visualization_msgs::Marker::SPHERE;
    covVelROS.action = visualization_msgs::Marker::ADD;
    covVelROS.pose.position.x = pose(0);
    covVelROS.pose.position.y = pose(1);
    covVelROS.pose.position.z = pose(2);
    q = R_to_quaternion(eigVec);
    covVelROS.pose.orientation.w = q(0);
    covVelROS.pose.orientation.x = q(1);
    covVelROS.pose.orientation.y = q(2);
    covVelROS.pose.orientation.z = q(3);      
    covVelROS.scale.x = sqrt(eigVal(0))*cov_scale;
    covVelROS.scale.y = sqrt(eigVal(1))*cov_scale;
    covVelROS.scale.z = sqrt(eigVal(2))*cov_scale;
    covVelROS.color.a = 0.4;
    covVelROS.color.r = r;
    covVelROS.color.g = g;
    covVelROS.color.b = b;
    covVelPub.publish(covVelROS);
  }

  // Color Coded Trajectory
  static colvec ppose = pose;
  static ros::Time pt = msg->header.stamp;
  ros::Time t = msg->header.stamp;
  if ((t - pt).toSec() > 0.5)
  {
    trajROS.header.frame_id = string("world");
    trajROS.header.stamp    = ros::Time::now();
    trajROS.ns              = string("trajectory");
    trajROS.type            = visualization_msgs::Marker::LINE_LIST;
    trajROS.action          = visualization_msgs::Marker::ADD;  
    trajROS.pose.position.x    = 0;
    trajROS.pose.position.y    = 0;
    trajROS.pose.position.z    = 0;  
    trajROS.pose.orientation.w = 1;  
    trajROS.pose.orientation.x = 0;  
    trajROS.pose.orientation.y = 0;  
    trajROS.pose.orientation.z = 0;
    trajROS.scale.x = 0.1;
    trajROS.scale.y = 0;
    trajROS.scale.z = 0;
    trajROS.color.r = 0.0;
    trajROS.color.g = 1.0;
    trajROS.color.b = 0.0;
    trajROS.color.a = 0.8;  
    geometry_msgs::Point p;          
    p.x = ppose(0);
    p.y = ppose(1);
    p.z = ppose(2);
    trajROS.points.push_back(p);   
    p.x = pose(0);
    p.y = pose(1);
    p.z = pose(2);
    trajROS.points.push_back(p);    
    std_msgs::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = 1;
    trajROS.colors.push_back(color);
    trajROS.colors.push_back(color);
    ppose = pose;
    pt = t;
    trajPub.publish(trajROS);
  }

  // Sensor availability
  sensorROS.header.frame_id = string("world");
  sensorROS.header.stamp    = msg->header.stamp;
  sensorROS.ns              = string("sensor");
  sensorROS.type            = visualization_msgs::Marker::TEXT_VIEW_FACING;
  sensorROS.action          = visualization_msgs::Marker::ADD;  
  sensorROS.pose.position.x = pose(0);
  sensorROS.pose.position.y = pose(1);
  sensorROS.pose.position.z = pose(2) + 1.0;
  sensorROS.pose.orientation.w = q(0);
  sensorROS.pose.orientation.x = q(1);
  sensorROS.pose.orientation.y = q(2);
  sensorROS.pose.orientation.z = q(3);      
  string strG = G?string(" GPS "):string("");
  string strV = V?string(" Vision "):string("");
  string strL = L?string(" Laser "):string("");
  sensorROS.text = "| " + strG + strV + strL + " |";
  sensorROS.color.a = 1.0;
  sensorROS.color.r = 1.0;
  sensorROS.color.g = 1.0;
  sensorROS.color.b = 1.0;
  sensorROS.scale.z = 0.5;
  sensorPub.publish(sensorROS);

  // Laser height measurement
  double H = msg->twist.covariance[32];
  heightROS.header.frame_id = string("height");
  heightROS.header.stamp = msg->header.stamp;
  heightROS.radiation_type = sensor_msgs::Range::ULTRASOUND;
  heightROS.field_of_view = 5.0 * M_PI / 180.0;
  heightROS.min_range = -100;
  heightROS.max_range =  100;
  heightROS.range     =  H; 
  heightPub.publish(heightROS);

  // Mesh model                                                  
  meshROS.header.frame_id = _frame_id;
  meshROS.header.stamp = msg->header.stamp; 
  meshROS.ns = "mesh";
  meshROS.id = 0;
  meshROS.type = visualization_msgs::Marker::MESH_RESOURCE;
  meshROS.action = visualization_msgs::Marker::ADD;
  meshROS.pose.position.x = msg->pose.pose.position.x;
  meshROS.pose.position.y = msg->pose.pose.position.y;
  meshROS.pose.position.z = msg->pose.pose.position.z;
  q(0) = msg->pose.pose.orientation.w;
  q(1) = msg->pose.pose.orientation.x;
  q(2) = msg->pose.pose.orientation.y;
  q(3) = msg->pose.pose.orientation.z;
  if (cross_config)
  {
    colvec ypr = R_to_ypr(quaternion_to_R(q));
    ypr(0)    += 45.0*PI/180.0;
    q          = R_to_quaternion(ypr_to_R(ypr)); 
  }  
  meshROS.pose.orientation.w = q(0);
  meshROS.pose.orientation.x = q(1);
  meshROS.pose.orientation.y = q(2);
  meshROS.pose.orientation.z = q(3);
  meshROS.scale.x = scale;
  meshROS.scale.y = scale;
  meshROS.scale.z = scale;
  meshROS.color.a = color_a;
  meshROS.color.r = color_r;
  meshROS.color.g = color_g;
  meshROS.color.b = color_b;
  meshROS.mesh_resource = mesh_resource;
  meshPub.publish(meshROS);                                                  

  // TF for raw sensor visualization
  // 按显示配置额外发布 base 下的 laser/vision/height 旋转坐标系；不替代真实标定外参。
  if (tf45)
  {
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(pose(0), pose(1), pose(2)));
    transform.setRotation(tf::Quaternion(q(1), q(2), q(3), q(0)));

    tf::Transform transform45;
    transform45.setOrigin(tf::Vector3(0, 0, 0));
    colvec y45 = zeros<colvec>(3);
    y45(0) = 45.0 * M_PI/180;
    colvec q45 = R_to_quaternion(ypr_to_R(y45));  
    transform45.setRotation(tf::Quaternion(q45(1), q45(2), q45(3), q45(0)));  

    tf::Transform transform90;
    transform90.setOrigin(tf::Vector3(0, 0, 0));
    colvec p90 = zeros<colvec>(3);
    p90(1) = 90.0 * M_PI/180;
    colvec q90 = R_to_quaternion(ypr_to_R(p90));  
    transform90.setRotation(tf::Quaternion(q90(1), q90(2), q90(3), q90(0)));  

    const ros::Time tf_stamp = ros::Time::now();
    broadcaster->sendTransform(tf::StampedTransform(transform,   tf_stamp, string("world"),  string("base")));      
    broadcaster->sendTransform(tf::StampedTransform(transform45, tf_stamp, string("base"), string("laser")));          
    broadcaster->sendTransform(tf::StampedTransform(transform45, tf_stamp, string("base"), string("vision")));          
    broadcaster->sendTransform(tf::StampedTransform(transform90, tf_stamp, string("base"), string("height")));          
  } 
}

/// @brief 读取颜色、网格资源、坐标系和显示开关，按全局 body_pose_topic 订阅位姿，再进入回调循环。
int main(int argc, char** argv)
{
  ros::init(argc, argv, "odom_visualization");
  ros::NodeHandle n("~");

  // 本段读取启动配置：下方“正常默认”来自当前 launch/XML，实际运行以参数服务器为准。
  // param() 的末项仅是缺参回退；getParam()/必需参数封装没有这一回退，具体见各行说明。
  // 参数读取：里程计显示 Marker 使用的网格 URI，默认继承上游 hummingbird 模型；Go2 本体另由 URDF 显
  // 示。
  // 配置位置：私有 /odom_visualization/mesh_resource；当前主 launch 未覆盖此键，取下面的缺参回退。
  // 缺参回退：std::string("package://odom_visualization/meshes/hummingbird.mesh")；只有参数服务器中
  // 没有该键时才使用。
  n.param("mesh_resource", mesh_resource, std::string("package://odom_visualization/meshes/hummingbird.mesh"));
  // 参数读取：里程计显示颜色的红通道，范围通常为 0~1。
  // 配置位置：私有 /odom_visualization/color/r；当前主 launch 未覆盖此键，取下面的缺参回退。
  // 缺参回退：0.0；只有参数服务器中没有该键时才使用。
  n.param("color/r", color_r, 0.0);
  // 参数读取：里程计显示颜色的绿通道，范围通常为 0~1。
  // 配置位置：私有 /odom_visualization/color/g；当前主 launch 未覆盖此键，取下面的缺参回退。
  // 缺参回退：0.0；只有参数服务器中没有该键时才使用。
  n.param("color/g", color_g, 0.0);
  // 参数读取：里程计显示颜色的蓝通道，范围通常为 0~1。
  // 配置位置：私有 /odom_visualization/color/b；当前主 launch 未覆盖此键，取下面的缺参回退。
  // 缺参回退：0.0；只有参数服务器中没有该键时才使用。
  n.param("color/b", color_b, 0.0);
  // 参数读取：里程计显示颜色的透明度，0=透明，1=不透明。
  // 配置位置：私有 /odom_visualization/color/a；当前主 launch 未覆盖此键，取下面的缺参回退。
  // 缺参回退：1.0；只有参数服务器中没有该键时才使用。
  n.param("color/a", color_a, 1.0);
  // 参数读取：里程计显示是否以首次位置为原点；影响本显示节点的数据，不修改规划器的目标坐标。
  // 配置位置：私有 /odom_visualization/origin；当前主 launch 未覆盖此键，取下面的缺参回退。
  // 缺参回退：false；只有参数服务器中没有该键时才使用。
  n.param("origin", origin, false);  
  // 参数读取：里程计 Marker 模型的显示缩放比例，不改变规划碰撞模型。
  // 配置位置：私有 /odom_visualization/robot_scale；当前主 launch 未覆盖此键，取下面的缺参回退。
  // 缺参回退：1.0；只有参数服务器中没有该键时才使用。
  n.param("robot_scale", scale, 1.0);
  // 参数读取：本节点输出消息所使用的世界参考系名称；名称设置本身不执行坐标变换。
  // 配置位置：私有 /odom_visualization/frame_id；当前主 launch 未覆盖此键，取下面的缺参回退。
  // 缺参回退：string("world")；只有参数服务器中没有该键时才使用。
  n.param("frame_id",   _frame_id, string("world") );    
 
  // 参数读取：里程计 Marker 的显示姿态修正选择，不改变机身状态反馈。
  // 配置位置：私有 /odom_visualization/cross_config；当前主 launch 未覆盖此键，取下面的缺参回退。
  // 缺参回退：false；只有参数服务器中没有该键时才使用。
  n.param("cross_config", cross_config, false);    
  // 参数读取：是否发布 world→base 及 laser/vision/height 等显示用 TF，默认 true。
  // 配置位置：私有 /odom_visualization/tf45；当前主 launch 未覆盖此键，取下面的缺参回退。
  // 缺参回退：true；只有参数服务器中没有该键时才使用。
  n.param("tf45", tf45, true);
  // 参数读取：协方差显示的缩放比例，不是滤波器噪声配置。
  // 配置位置：私有 /odom_visualization/covariance_scale；当前主 launch 未覆盖此键，取下面的缺参回退
  // 。
  // 缺参回退：100.0；只有参数服务器中没有该键时才使用。
  n.param("covariance_scale",    cov_scale,  100.0);
  // 参数读取：是否绘制位置协方差显示对象。
  // 配置位置：私有 /odom_visualization/covariance_position；当前主 launch 未覆盖此键，取下面的缺参
  // 回退。
  // 缺参回退：false；只有参数服务器中没有该键时才使用。
  n.param("covariance_position", cov_pos,    false);    
  // 参数读取：是否绘制速度协方差显示对象。
  // 配置位置：私有 /odom_visualization/covariance_velocity；当前主 launch 未覆盖此键，取下面的缺参
  // 回退。
  // 缺参回退：false；只有参数服务器中没有该键时才使用。
  n.param("covariance_velocity", cov_vel,    false);    
  // 参数读取：是否根据协方差改变显示颜色。
  // 配置位置：私有 /odom_visualization/covariance_color；当前主 launch 未覆盖此键，取下面的缺参回退
  // 。
  // 缺参回退：false；只有参数服务器中没有该键时才使用。
  n.param("covariance_color",    cov_color,  false);    
  
  std::string body_pose_topic;
  // 参数读取：机身 Odometry 接口；默认仿真 /quad_0/body_pose，真机由父 launch 切到
  // /LIO/odom_vehicle。
  // 配置来源：run.launch → /body_pose_topic；正常默认启动取 /quad_0/body_pose。
  // 缺参回退：std::string("/quad_0/body_pose")；只有参数服务器中没有该键时才使用。
  ros::param::param<std::string>("/body_pose_topic", body_pose_topic, std::string("/quad_0/body_pose"));
  // 运行连接（接收入口）：机身 Odometry 接口，实际名称由 /body_pose_topic 指定；默认仿真
  // /quad_0/body_pose，接入规划/控制/感知/显示相应回调。
  // 队列容量：100 条消息，不是发布/规划频率。 到达后交给 odom_callback。
  ros::Subscriber sub_odom = n.subscribe(body_pose_topic, 100, odom_callback);
  // 运行连接（输出入口）：里程计显示节点的当前位姿输出，默认 /odom_visualization/pose。
  // 队列容量：100 条消息，不是发布/规划频率。 锁存最后一条消息，供新连接的订阅者立即取得。
  posePub   = n.advertise<geometry_msgs::PoseStamped>("pose",                100, true);
  // 运行连接（输出入口）：里程计显示节点的历史实际路径输出，默认 /odom_visualization/path，用于与规
  // 划轨迹对照。
  // 队列容量：100 条消息，不是发布/规划频率。 锁存最后一条消息，供新连接的订阅者立即取得。
  pathPub   = n.advertise<nav_msgs::Path>(            "path",                100, true);
  // 运行连接（输出入口）：里程计显示节点的速度 Marker，默认 /odom_visualization/velocity。
  // 队列容量：100 条消息，不是发布/规划频率。 锁存最后一条消息，供新连接的订阅者立即取得。
  velPub    = n.advertise<visualization_msgs::Marker>("velocity",            100, true);
  // 运行连接（输出入口）：里程计显示节点的位置不确定性 Marker，是否更新由显示开关控制。
  // 队列容量：100 条消息，不是发布/规划频率。 锁存最后一条消息，供新连接的订阅者立即取得。
  covPub    = n.advertise<visualization_msgs::Marker>("covariance",          100, true);
  // 运行连接（输出入口）：里程计显示节点的速度不确定性 Marker，属于显示数据而非滤波器输入。
  // 队列容量：100 条消息，不是发布/规划频率。 锁存最后一条消息，供新连接的订阅者立即取得。
  covVelPub = n.advertise<visualization_msgs::Marker>("covariance_velocity", 100, true);
  // 运行连接（输出入口）：里程计显示节点的辅助轨迹 Marker，与 FSM 发布的可执行 Bspline 分开。
  // 队列容量：100 条消息，不是发布/规划频率。 锁存最后一条消息，供新连接的订阅者立即取得。
  trajPub   = n.advertise<visualization_msgs::Marker>("trajectory",          100, true);
  // 运行连接（输出入口）：里程计显示节点的辅助传感器 Marker，不提供真实传感器标定。
  // 队列容量：100 条消息，不是发布/规划频率。 锁存最后一条消息，供新连接的订阅者立即取得。
  sensorPub = n.advertise<visualization_msgs::Marker>("sensor",              100, true);
  // 运行连接（输出入口）：里程计显示节点的网格 Marker，Go2 的关节模型另由 robot_state_publisher 显
  // 示。
  // 队列容量：100 条消息，不是发布/规划频率。 锁存最后一条消息，供新连接的订阅者立即取得。
  meshPub   = n.advertise<visualization_msgs::Marker>("robot",               100, true);  
  // 运行连接（输出入口）：里程计显示节点的高度显示接口（Range）。
  // 队列容量：100 条消息，不是发布/规划频率。 锁存最后一条消息，供新连接的订阅者立即取得。
  heightPub = n.advertise<sensor_msgs::Range>(        "height",              100, true);  
  tf::TransformBroadcaster b;
  broadcaster = &b;

  ros::spin();

  return 0;
}
