/// @file
/// 可选 OpenGL 感知后端，编译需要 USE_GPU=ON，运行还需 simulator.xml 选择 use_gpu。
/// 节点名沿用 pcl_render_node，便于保持下游默认话题接口；CPU 和 GPU 后端是互斥替代分支。
/// 激光分支使用 OpenGL 渲染组件；深度分支仍有基于点云的针孔合成流程，应分别追踪实际调用。
#include "opengl_sim.hpp"
#include <cv_bridge/cv_bridge.h>
#include <algorithm>
#include <cmath>
#include <deque>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <numeric>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <tr1/unordered_map>
#include <tf2_ros/transform_broadcaster.h>

using namespace Eigen;
using namespace std;

#define MAX_INTENSITY 1
#define MIN_INTENSITY 0.1

string file_name;
deque<double> comp_time_vec;

std::string quad_name;
std::string sensor_type_ = "depth";
int drone_num = 0;
int drone_id = 0;
double lidar_pitch;
opengl_pointcloud_render render;

ros::Publisher pub_cloud, pub_intercloud, pub_dyncloud, pub_uavcloud, pinhole_depth_pub_,
    camera_pose_pub_, lidar_pose_pub_, comp_time_pub, pub_collisioncloud;
sensor_msgs::PointCloud2 local_map_pcl;
sensor_msgs::PointCloud2 local_depth_pcl;
ros::Subscriber odom_sub;
ros::Subscriber global_map_sub, local_map_sub;
ros::Timer local_sensing_timer, pose_timer, dynobj_timer;

ros::Time t_init;

// 初始 false；收到机身状态后 true，与 has_global_map 一起作为开始渲染的就绪条件，不含数据超时判定。
bool has_odom(false);
// 初始 false；全局场景准备成功后 true，不能解释为规划器已经融合了全部环境。
bool has_global_map(false);
// 私有 use_global_map_topic 缺省 false；simulator.xml 在随机地图分支将它设 true，从话题加载场景。
bool use_global_map_topic(false);

// 最近机身状态，由机身位姿回调更新；感知定时器据此设置观测位置和朝向。
nav_msgs::Odometry odom_;
// sensor2body：相机安装变换；sensor2world：机身变换组合后的相机世界位姿，随观测更新供深度投影。
Eigen::Matrix4f sensor2body, sensor2world;

std::string quadTopic(const std::string &name) {
  std::string prefix = quad_name.empty() ? std::string("quad_0") : quad_name;
  if (prefix.front() != '/') {
    prefix = "/" + prefix;
  }
  return prefix + "/" + name;
}

// 私有 output_pcd 缺省 0，调试文件输出开关。
int output_pcd;
// 私有 collisioncheck_enable 缺省 0；仅控制仿真器独立碰撞报告，不影响规划器安全定时器。
int collisioncheck_enable;
// 私有 is_360lidar 缺省 1；全周模式影响雷达角度图尺寸及视场筛选。
int is_360lidar;
int use_avia_pattern, use_vlp32_pattern, use_minicf_pattern, use_os128_pattern, use_os64_pattern,
use_gaussian_filter;
double livox_linestep;
double sensing_horizon, sensing_rate, estimation_rate, polar_resolution, yaw_fov, vertical_fov,
min_raylength, downsample_res, curvature_limit, hash_cubesize, collision_range;
double x_size, y_size, z_size;
double gl_xl, gl_yl, gl_zl;
double resolution, inv_resolution;
int GLX_SIZE, GLY_SIZE, GLZ_SIZE;

// 合成深度图宽度，私有 cam_width 缺省 640 px。
int cam_width = 640;
// 合成深度图高度，私有 cam_height 缺省 480 px。
int cam_height = 480;
// 针孔水平焦距，私有缺省约 387.229 px，用于世界场景点投影。
double cam_fx = 387.229248046875;
// 针孔垂直焦距，私有缺省约 387.229 px。
double cam_fy = 387.229248046875;
// 针孔主点水平位置，私有缺省约 321.046 px。
double cam_cx = 321.04638671875;
// 针孔主点垂直位置，私有缺省约 243.450 px；图像缩放后需与其他内参一起对应。
double cam_cy = 243.44969177246094;

int plane_interline = 1;

// multi uav variables
// int drone_id = 0;
vector<Eigen::Vector3d> other_uav_pos;
vector<double> other_uav_rcv_time;
vector<vector<PointType>> otheruav_points, otheruav_points_inrender;
vector<vector<int>> otheruav_pointsindex, otheruav_pointsindex_inrender;
pcl::PointCloud<PointType> otheruav_points_vis, dyn_points_vis;
double uav_size[3];
int uav_points_num;
// int drone_num = 0;
int drone_drawpoints_num[3];

int use_uav_extra_model = 1;
pcl::PointCloud<PointType> uav_extra_model;

// dynamic objects variables
// 私有 dynobj_enable 缺省 0；启用后维护模拟障碍运动，并不代表规划器预测未来障碍轨迹。
int dynobj_enable;
// 动态障碍的尺寸尺度（m），由私有配置读取，具体几何随物体类型决定。
double dynobject_size;
vector<Eigen::Vector3d> dynobj_poss;
vector<int> dynobj_move_modes;
vector<int> dynobj_types;
vector<Eigen::Vector3d> dynobj_dir;
vector<PointType> dynobj_points;
pcl::PointCloud<PointType> dynobj_points_vis;
vector<int> dynobj_pointsindex;
// 动态障碍速度尺度（m/s），由私有配置读取。
double dyn_velocity;
// 动态障碍数量，私有缺省 0。
int dynobject_num;
int dyn_mode;
Eigen::Vector3f map_min, map_max;
int dyn_obs_diff_size_on = 1;
vector<double> dyn_obs_size_vec;
// 各模拟障碍的运动参考起始时刻，用于动态场景生成。
vector<ros::Time> dyn_start_time_vec;
PointType global_mapmin;
PointType global_mapmax;

pcl::PointCloud<PointType> dynamic_input_points;

pcl::PointCloud<PointType> cloud_all_map, local_map_filled, point_in_sensor;
pcl::PointCloud<PointType>::Ptr local_map(new pcl::PointCloud<PointType>);
// pcl::VoxelGrid<PointType> _voxel_sampler;
sensor_msgs::PointCloud2 local_map_pcd, sensor_map_pcd;

// variables for collision checks
pcl::search::KdTree<PointType> _kdtreeLocalMap, kdtree_dyn;
vector<int> pointIdxRadiusSearch;
vector<float> pointRadiusSquaredDistance;
double collision_check_time_sum = 0;
int collision_check_time_count = 0;
// hash map of uav models for collision checks
std::tr1::unordered_map<int, bool> uavpoint_hashmap;
PointType uav_modelmin;
PointType uav_modelmax;
int uavhash_xsize, uavhash_ysize, uavhash_zsize;
pcl::PointCloud<PointType> collision_points;

/// @brief 选择深度图输出分支。
bool useDepthSensor() {
  return sensor_type_ == "depth";
}

/// @brief 选择雷达点云输出分支。
bool useLidarSensor() {
  return sensor_type_ == "lidar";
}

nav_msgs::Odometry makeSensorPoseMsg(const nav_msgs::Odometry &body_odom,
                                     const Eigen::Matrix4f &sensor_pose,
                                     const std::string &child_frame_id) {
  Eigen::Quaternionf q(sensor_pose.block<3, 3>(0, 0));
  q.normalize();

  nav_msgs::Odometry sensor_odom;
  sensor_odom.header = body_odom.header;
  sensor_odom.header.frame_id = "world";
  sensor_odom.child_frame_id = child_frame_id;
  sensor_odom.pose.pose.position.x = sensor_pose(0, 3);
  sensor_odom.pose.pose.position.y = sensor_pose(1, 3);
  sensor_odom.pose.pose.position.z = sensor_pose(2, 3);
  sensor_odom.pose.pose.orientation.w = q.w();
  sensor_odom.pose.pose.orientation.x = q.x();
  sensor_odom.pose.pose.orientation.y = q.y();
  sensor_odom.pose.pose.orientation.z = q.z();
  return sensor_odom;
}

/// @brief 准备针孔投影所需场景点云与近邻索引，返回地图是否可用。
bool preparePinholeMap(const pcl::PointCloud<PointType> &raw_cloud) {
  pcl::PointCloud<PointType> finite_cloud;
  finite_cloud.points.reserve(raw_cloud.points.size());
  for (const auto &point : raw_cloud.points) {
    if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
      finite_cloud.points.push_back(point);
    }
  }

  if (finite_cloud.empty()) {
    ROS_ERROR("[opengl_render_node] Empty map cloud; cannot publish pinhole depth.");
    return false;
  }

  finite_cloud.width = finite_cloud.points.size();
  finite_cloud.height = 1;
  finite_cloud.is_dense = true;

  pcl::VoxelGrid<PointType> voxel_sampler;
  voxel_sampler.setInputCloud(finite_cloud.makeShared());
  voxel_sampler.setLeafSize(downsample_res, downsample_res, downsample_res);
  voxel_sampler.filter(cloud_all_map);

  if (cloud_all_map.empty()) {
    ROS_ERROR("[opengl_render_node] Downsampled map cloud is empty; cannot publish pinhole depth.");
    return false;
  }

  cloud_all_map.width = cloud_all_map.points.size();
  cloud_all_map.height = 1;
  cloud_all_map.is_dense = true;

  pcl::getMinMax3D(cloud_all_map, global_mapmin, global_mapmax);
  _kdtreeLocalMap.setInputCloud(cloud_all_map.makeShared());

  ROS_INFO("[opengl_render_node] Pinhole depth map ready, points: %zu", cloud_all_map.points.size());
  return true;
}

/// @brief 从 PCD 准备针孔/碰撞分支所需地图，失败时由 main() 中止相关启动流程。
bool loadPinholeMapFromFile(const std::string &map_file) {
  pcl::PointCloud<PointType> raw_cloud;
  if (pcl::io::loadPCDFile<PointType>(map_file, raw_cloud) == -1) {
    ROS_ERROR_STREAM("[opengl_render_node] Failed to read PCD file for pinhole depth: " << map_file);
    return false;
  }

  return preparePinholeMap(raw_cloud);
}

/// @brief 将当前相机变换封装为 Odometry，与深度图的时戳和坐标解释对应。
void publishCameraPose(const nav_msgs::Odometry &body_odom) {
  camera_pose_pub_.publish(makeSensorPoseMsg(body_odom, sensor2world, "camera"));
}

/// @brief 按渲染使用的位置与姿态发布雷达位姿，避免观测与位姿方向配置分离。
void publishLidarPose(const nav_msgs::Odometry &body_odom, const ros::Time &stamp,
                      const Eigen::Vector3f &pos, const Eigen::Quaternionf &quat) {
  nav_msgs::Odometry lidar_pose;
  lidar_pose.header = body_odom.header;
  lidar_pose.header.stamp = stamp;
  lidar_pose.header.frame_id = "world";
  lidar_pose.child_frame_id = "lidar";
  lidar_pose.pose.pose.position.x = pos.x();
  lidar_pose.pose.pose.position.y = pos.y();
  lidar_pose.pose.pose.position.z = pos.z();
  lidar_pose.pose.pose.orientation.x = quat.x();
  lidar_pose.pose.pose.orientation.y = quat.y();
  lidar_pose.pose.pose.orientation.z = quat.z();
  lidar_pose.pose.pose.orientation.w = quat.w();
  lidar_pose_pub_.publish(lidar_pose);
}

/// @brief 在相机坐标系按内参投影附近场景点，对像素保留近端深度，发布米制 32FC1 图像。
void publishPinholeDepth(const ros::Time &stamp) {
  if (cloud_all_map.empty()) {
    ROS_WARN_THROTTLE(1.0, "[opengl_render_node] No pinhole map cloud yet; skip depth publish.");
    return;
  }

  if (cam_width <= 0 || cam_height <= 0 || cam_fx <= 0.0 || cam_fy <= 0.0) {
    ROS_WARN_THROTTLE(1.0, "[opengl_render_node] Invalid pinhole camera intrinsics.");
    return;
  }

  cv::Mat depth_image(cam_height, cam_width, CV_32FC1, cv::Scalar(0.0f));

  const Eigen::Matrix4f world2camera = sensor2world.inverse();
  const Eigen::Vector3f camera_pos(sensor2world(0, 3), sensor2world(1, 3), sensor2world(2, 3));
  const double max_depth = std::max(0.1, sensing_horizon);

  PointType search_point;
  search_point.x = camera_pos.x();
  search_point.y = camera_pos.y();
  search_point.z = camera_pos.z();

  std::vector<int> indices;
  std::vector<float> distances;
  _kdtreeLocalMap.radiusSearch(search_point, max_depth, indices, distances);

  for (const int index : indices) {
    const PointType &pt = cloud_all_map.points[index];
    const Eigen::Vector4f pt_world(pt.x, pt.y, pt.z, 1.0f);
    const Eigen::Vector4f pt_camera = world2camera * pt_world;

    const double z = pt_camera.z();
    if (z <= 1e-3 || z > max_depth) {
      continue;
    }

    const int u_center = static_cast<int>(std::round(cam_fx * pt_camera.x() / z + cam_cx));
    const int v_center = static_cast<int>(std::round(cam_fy * pt_camera.y() / z + cam_cy));
    if (u_center < 0 || u_center >= cam_width || v_center < 0 || v_center >= cam_height) {
      continue;
    }

    const int splat_radius =
        std::min(4, std::max(0, static_cast<int>(std::ceil(0.5 * cam_fx * downsample_res / z))));
    const int u_min = std::max(0, u_center - splat_radius);
    const int u_max = std::min(cam_width - 1, u_center + splat_radius);
    const int v_min = std::max(0, v_center - splat_radius);
    const int v_max = std::min(cam_height - 1, v_center + splat_radius);

    for (int v = v_min; v <= v_max; ++v) {
      for (int u = u_min; u <= u_max; ++u) {
        float &depth = depth_image.at<float>(v, u);
        if (depth <= 0.0f || z < depth) {
          depth = static_cast<float>(z);
        }
      }
    }
  }

  cv_bridge::CvImage out_msg;
  out_msg.header.stamp = stamp;
  out_msg.header.frame_id = "camera";
  out_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  out_msg.image = depth_image;
  pinhole_depth_pub_.publish(out_msg.toImageMsg());
}

pcl::PointCloud<PointType> generate_sphere_cloud(double size) {
  pcl::PointCloud<PointType> sphere_cloud;
  PointType temp_point;
  double radius = size / 2;
  double x, y, z;

  // use downsample resolution to compute how many points in a sphere
  int theta_num = 2 * M_PI * radius / downsample_res;
  int fi_num = M_PI * radius / downsample_res;

  for (int i = 0; i < theta_num; i++) {
    for (int j = 0; j < fi_num; j++) {
      x = radius * sin(i * 2 * M_PI / theta_num) * cos(j * M_PI / fi_num);
      y = radius * sin(i * 2 * M_PI / theta_num) * sin(j * M_PI / fi_num);
      z = radius * cos(i * 2 * M_PI / theta_num);
      temp_point.x = x;
      temp_point.y = y;
      temp_point.z = z;
      temp_point.intensity = MIN_INTENSITY + (MAX_INTENSITY - MIN_INTENSITY) * 1.0 / 2.0;
      sphere_cloud.push_back(temp_point);
    }
  }

  return sphere_cloud;
}

pcl::PointCloud<PointType> generate_box_cloud(double size) {
  pcl::PointCloud<PointType> box_cloud;
  PointType temp_point;
  double x, y, z;

  // use downsample resolution to compute how many points in a box
  int x_num = size / downsample_res;
  int y_num = size / downsample_res;
  int z_num = size / downsample_res;

  // for (int i = 0; i < x_num; i++)
  // {
  //   for (int j = 0; j < y_num; j++)
  //   {
  //     for (int k = 0; k < z_num; k++)
  //     {
  //       x = i * downsample_res - size / 2;
  //       y = j * downsample_res - size / 2;
  //       z = k * downsample_res - size / 2;
  //       temp_point.x = x;
  //       temp_point.y = y;
  //       temp_point.z = z;
  //       temp_point.intensity = MIN_INTENSITY + (MAX_INTENSITY - MIN_INTENSITY) * 2.0 / 2.0;
  //       box_cloud.push_back(temp_point);
  //     }
  //   }
  // }

  // draw 2 xy plane
  for (int i = 0; i <= x_num; i++) {
    for (int j = 0; j <= y_num; j++) {
      x = i * downsample_res - size / 2;
      y = j * downsample_res - size / 2;
      z = -size / 2;
      temp_point.x = x;
      temp_point.y = y;
      temp_point.z = z;
      temp_point.intensity = MIN_INTENSITY + (MAX_INTENSITY - MIN_INTENSITY) * 2.0 / 2.0;
      box_cloud.push_back(temp_point);
      z = size / 2;
      temp_point.z = z;
      temp_point.intensity = MIN_INTENSITY + (MAX_INTENSITY - MIN_INTENSITY) * 2.0 / 2.0;
      box_cloud.push_back(temp_point);
    }
  }
  // draw 4 plane
  for (int k = 1; k < z_num; k++) {
    // draw x two lines
    for (int i = 0; i <= x_num; i++) {
      x = i * downsample_res - size / 2;
      y = -size / 2;
      z = k * downsample_res - size / 2;
      temp_point.x = x;
      temp_point.y = y;
      temp_point.z = z;
      temp_point.intensity = MIN_INTENSITY + (MAX_INTENSITY - MIN_INTENSITY) * 2.0 / 2.0;
      box_cloud.push_back(temp_point);

      x = i * downsample_res - size / 2;
      y = size / 2;
      z = k * downsample_res - size / 2;
      temp_point.x = x;
      temp_point.y = y;
      temp_point.z = z;
      temp_point.intensity = MIN_INTENSITY + (MAX_INTENSITY - MIN_INTENSITY) * 2.0 / 2.0;
      box_cloud.push_back(temp_point);
    }
    for (int j = 0; j <= y_num; j++) {
      x = -size / 2;
      y = j * downsample_res - size / 2;
      z = k * downsample_res - size / 2;
      temp_point.x = x;
      temp_point.y = y;
      temp_point.z = z;
      temp_point.intensity = MIN_INTENSITY + (MAX_INTENSITY - MIN_INTENSITY) * 2.0 / 2.0;
      box_cloud.push_back(temp_point);

      x = size / 2;
      y = j * downsample_res - size / 2;
      z = k * downsample_res - size / 2;
      temp_point.x = x;
      temp_point.y = y;
      temp_point.z = z;
      temp_point.intensity = MIN_INTENSITY + (MAX_INTENSITY - MIN_INTENSITY) * 2.0 / 2.0;
      box_cloud.push_back(temp_point);
    }
  }
  return box_cloud;
}

/// @brief 生成可选障碍几何点云，供动态场景输入渲染。
void generate_ptclouds_by_pos(Eigen::Vector3d obs_pos, int obs_type,
                              pcl::PointCloud<PointType> &obs_cloud,
                              vector<PointType> &obs_points) {
  // 0 for using uav model, 1 for using sphere model, 2 for using box model
  switch (obs_type) {
  case 0:
    if (use_uav_extra_model) {
      obs_cloud = uav_extra_model;
      for (int i = 0; i < obs_cloud.size(); i++) {
        obs_cloud.points[i].intensity = MIN_INTENSITY + (MAX_INTENSITY - MIN_INTENSITY) * 0.0 / 2.0;
      }
    } else {
      obs_cloud = generate_box_cloud(dynobject_size);
    }
    break;

  case 1:
    obs_cloud = generate_sphere_cloud(dynobject_size);
    break;

  case 2:
    obs_cloud = generate_box_cloud(dynobject_size);
    break;

  default:
    break;
  }

  for (int i = 0; i < obs_cloud.size(); i++) {
    obs_cloud.points[i].x = obs_cloud.points[i].x + obs_pos(0);
    obs_cloud.points[i].y = obs_cloud.points[i].y + obs_pos(1);
    obs_cloud.points[i].z = obs_cloud.points[i].z + obs_pos(2);
    obs_points.push_back(obs_cloud.points[i]);
  }
}

/// @brief 推进可选动态障碍仿真并组织渲染点云，与规划器是否具有动态预测能力分别判断。
void dynobjGenerate(const ros::TimerEvent &event) {
  if (dynobj_enable == 1) {
    dynobj_points.clear();
    dynobj_pointsindex.clear();
    dynobj_points_vis.clear();
    PointType temp_point;
    int dynpt_count = 0;
    double fly_time;
    Eigen::Vector3d gravity_vec;
    gravity_vec << 0, 0, -1;
    Eigen::Vector3d dyntemp_dir_polar;

    // rewrite generate dynamic obstacles
    for (int n = 0; n < dynobject_num; n++) {
      // according to motion mode, compute the positions of dynamic obstacle
      switch (dynobj_move_modes[n]) {
      case 0: {
        // constant speed mode
        fly_time = (ros::Time::now() - dyn_start_time_vec[n]).toSec();
        dynobj_poss[n] = dynobj_poss[n] + fly_time * dynobj_dir[n];
        break;
      }

      case 1: {
        // constant gravity acceleration mode
        fly_time = (ros::Time::now() - dyn_start_time_vec[n]).toSec();
        dynobj_dir[n] = dynobj_dir[n] + gravity_vec * fly_time;
        dynobj_poss[n] = dynobj_poss[n] + fly_time * dynobj_dir[n];
        break;
      }

      case 2: {
        // random walk mode
        fly_time = (ros::Time::now() - dyn_start_time_vec[n]).toSec();
        dyntemp_dir_polar(0) = rand() / double(RAND_MAX) * 3.1415926;
        dyntemp_dir_polar(1) = (rand() / double(RAND_MAX) - 0.5) * 3.1415926;
        dynobj_dir[n](0) = dyn_velocity * sin(dyntemp_dir_polar(1));
        dynobj_dir[n](1) = dyn_velocity * cos(dyntemp_dir_polar(1)) * sin(dyntemp_dir_polar(0));
        dynobj_dir[n](2) = dyn_velocity * cos(dyntemp_dir_polar(1)) * cos(dyntemp_dir_polar(0));
        dynobj_poss[n] = dynobj_poss[n] + fly_time * dynobj_dir[n];
        break;
      }

      default:
        break;
      }

      // if dynamic obstacle is out of bounding box, regenerte one
      if (dynobj_poss[n](0) < map_min(0) || dynobj_poss[n](0) > map_max(0) ||
          dynobj_poss[n](1) < map_min(1) || dynobj_poss[n](1) > map_max(1) ||
          dynobj_poss[n](2) < map_min(2) || dynobj_poss[n](2) > map_max(2)) {
        dynobj_poss[n](0) = rand() / double(RAND_MAX) * (map_max(0) - map_min(0)) + map_min(0);
        dynobj_poss[n](1) = rand() / double(RAND_MAX) * (map_max(1) - map_min(1)) + map_min(1);
        dynobj_poss[n](2) = rand() / double(RAND_MAX) * (map_max(2) - map_min(2)) + map_min(2);
        Eigen::Vector3d dyntemp_dir_polar;
        dyntemp_dir_polar(0) = rand() / double(RAND_MAX) * 3.1415926;
        dyntemp_dir_polar(1) = (rand() / double(RAND_MAX) - 0.5) * 3.1415926;
        dynobj_dir[n](0) = dyn_velocity * sin(dyntemp_dir_polar(1));
        dynobj_dir[n](1) = dyn_velocity * cos(dyntemp_dir_polar(1)) * sin(dyntemp_dir_polar(0));
        dynobj_dir[n](2) = dyn_velocity * cos(dyntemp_dir_polar(1)) * cos(dyntemp_dir_polar(0));
      }
      dyn_start_time_vec[n] = ros::Time::now();
      // generate point cloud of dynamic obstacle
      pcl::PointCloud<PointType> dynobj_cloud;
      vector<PointType> temp_dynobj_points;
      generate_ptclouds_by_pos(dynobj_poss[n], dynobj_types[n], dynobj_cloud, temp_dynobj_points);
      dynobj_points_vis = dynobj_points_vis + dynobj_cloud;
      dynobj_points.insert(dynobj_points.end(), temp_dynobj_points.begin(),
                           temp_dynobj_points.end());
      // for (int i = 0; i < temp_dynobj_points.size(); i++)
      // {
      //   dynobj_pointsindex.push_back(dynpt_count + origin_mapptcount);
      //   dynpt_count++;
      // }
    }

    ROS_INFO("dynobj points size = %d", dynobj_points_vis.points.size());

    dynobj_points_vis.width = dynobj_points_vis.points.size();
    dynobj_points_vis.height = 1;
    dynobj_points_vis.is_dense = true;

    // sensor_msgs::PointCloud2 dynobj_points_pcd;
    // pcl::toROSMsg(dynobj_points_vis, dynobj_points_pcd);
    // dynobj_points_pcd.header = odom_.header;
    // dynobj_points_pcd.header.frame_id = "world";
    // pub_dyncloud.publish(dynobj_points_pcd);

    kdtree_dyn.setInputCloud(dynobj_points_vis.makeShared());
    // has_dyn_map = true;

    // dyn_start_time = ros::Time::now();
  }
}

/// @brief 缓存机身状态并更新相机变换及可选碰撞信息；激光位姿在渲染分支配套发布。
void rcvOdometryCallback(const nav_msgs::Odometry &odom) {
  /*if(!has_global_map)
    return;*/
  has_odom = true;
  odom_ = odom;

  Matrix4f body2world = Matrix4f::Identity();

  Eigen::Vector3f request_position;
  Eigen::Quaternionf pose;
  pose.x() = odom.pose.pose.orientation.x;
  pose.y() = odom.pose.pose.orientation.y;
  pose.z() = odom.pose.pose.orientation.z;
  pose.w() = odom.pose.pose.orientation.w;
  body2world.block<3, 3>(0, 0) = pose.toRotationMatrix();
  body2world(0, 3) = odom.pose.pose.position.x;
  body2world(1, 3) = odom.pose.pose.position.y;
  body2world(2, 3) = odom.pose.pose.position.z;

  // convert to cam pose
  sensor2world = body2world * sensor2body;

  // collision check function
  // running time statistics
  ros::Time t1 = ros::Time::now();

  if (collisioncheck_enable) {
    collision_points.points.clear();
    PointType searchPoint;
    searchPoint.x = odom.pose.pose.position.x;
    searchPoint.y = odom.pose.pose.position.y;
    searchPoint.z = odom.pose.pose.position.z;
    if (_kdtreeLocalMap.radiusSearch(searchPoint, collision_range, pointIdxRadiusSearch,
                                     pointRadiusSquaredDistance) > 0) {
      if (use_uav_extra_model) {
        // check if the neighbor points are in the uav model
        bool collision = false;
        for (int i = 0; i < pointIdxRadiusSearch.size(); i++) {
          int index = pointIdxRadiusSearch[i];
          PointType p = cloud_all_map.points[index];

          // transform the point to the uav model coordinate
          Eigen::Vector3f p_eigen(p.x, p.y, p.z);
          p_eigen =
          body2world.block<3, 3>(0, 0).transpose() * (p_eigen - body2world.block<3, 1>(0, 3));

          int hash_x = (p_eigen(0) - uav_modelmin.x) / downsample_res;
          int hash_y = (p_eigen(1) - uav_modelmin.y) / downsample_res;
          int hash_z = (p_eigen(2) - uav_modelmin.z) / downsample_res;
          int hash_index = hash_x + hash_y * uavhash_xsize + hash_z * uavhash_xsize * uavhash_ysize;
          if (uavpoint_hashmap.find(hash_index) != uavpoint_hashmap.end()) {
            collision = true;
            ROS_ERROR("ENVIRONMENT COLLISION DETECTED!!!");
            // break;
            collision_points.points.push_back(p);
          }
        }
      } else {
        ROS_ERROR("ENVIRONMENT COLLISION DETECTED!!!");
      }
    }
    if (dynobj_enable || drone_num > 1) {
      if (kdtree_dyn.radiusSearch(searchPoint, collision_range, pointIdxRadiusSearch,
                                  pointRadiusSquaredDistance) > 0) {
        if (use_uav_extra_model) {
          // check if the neighbor points are in the uav model
          bool collision = false;
          for (int i = 0; i < pointIdxRadiusSearch.size(); i++) {
            int index = pointIdxRadiusSearch[i];
            PointType p = cloud_all_map.points[index];
            int hash_x = (p.x - uav_modelmin.x) / downsample_res;
            int hash_y = (p.y - uav_modelmin.y) / downsample_res;
            int hash_z = (p.z - uav_modelmin.z) / downsample_res;
            int hash_index =
            hash_x + hash_y * uavhash_xsize + hash_z * uavhash_xsize * uavhash_ysize;
            if (uavpoint_hashmap.find(hash_index) != uavpoint_hashmap.end()) {
              collision = true;
              ROS_ERROR("DYNAMIC OBSTACLES COLLISION DETECTED!!!");
              // break;
              collision_points.points.push_back(p);
            }
          }
        } else {
          ROS_ERROR("DYNAMIC OBSTACLES COLLISION DETECTED!!!");
        }
      }
    }
  }

  // publish collision points
  sensor_msgs::PointCloud2 collision_points_vis_pcd;
  pcl::toROSMsg(collision_points, collision_points_vis_pcd);
  collision_points_vis_pcd.header.frame_id = "world";
  collision_points_vis_pcd.header.stamp = ros::Time::now();
  pub_collisioncloud.publish(collision_points_vis_pcd);

  ros::Time t2 = ros::Time::now();
  collision_check_time_sum += (t2 - t1).toSec();
  collision_check_time_count++;
  if (collision_check_time_count == 100) {
    // ROS_INFO("collision check time: %f", collision_check_time_sum / collision_check_time_count);
    collision_check_time_sum = 0;
    collision_check_time_count = 0;
  }
}

// decentralize simulation, get other uav odometry and generate point clouds
/// @brief 缓存其他机器人状态，供多机几何或显示点云分支使用。
void multiOdometryCallback(const nav_msgs::OdometryConstPtr &msg, int drone_id) {
  Eigen::Vector3d uav_pos;
  uav_pos(0) = msg->pose.pose.position.x;
  uav_pos(1) = msg->pose.pose.position.y;
  uav_pos(2) = msg->pose.pose.position.z;
  other_uav_pos[drone_id] = uav_pos;
  other_uav_rcv_time[drone_id] = msg->header.stamp.toSec();

  otheruav_points[drone_id].clear();
  otheruav_pointsindex[drone_id].clear();
  otheruav_points_vis.clear();

  PointType temp_point;
  int uavpt_count = 0;

  if (use_uav_extra_model == 0) {
    // generate uav point cloud
    double x_min = uav_pos(0) - 0.5 * uav_size[0];
    double y_min = uav_pos(1) - 0.5 * uav_size[1];
    double z_min = uav_pos(2) - 0.5 * uav_size[2];

    for (int i = 0; i < drone_drawpoints_num[0]; i++) {
      for (int j = 0; j < drone_drawpoints_num[1]; j++) {
        for (int k = 0; k < drone_drawpoints_num[2]; k++) {
          temp_point.x = i * downsample_res + x_min;
          temp_point.y = j * downsample_res + y_min;
          temp_point.z = k * downsample_res + z_min;
          temp_point.intensity =
          ((float)(MAX_INTENSITY - MIN_INTENSITY)) * ((drone_id + 1.0) / (float(drone_num))) +
          MIN_INTENSITY; // set the intensity of the point
          otheruav_points[drone_id].push_back(temp_point);
          // otheruav_pointsindex[drone_id].push_back(uavpt_count + origin_mapptcount + 100000 +
          // drone_id * uav_points_num);
          otheruav_points_vis.push_back(temp_point);
          uavpt_count++;
        }
      }
    }
  } else {
    // read pcd model from file and attach it to UAVs odom
    Eigen::Quaterniond pose;
    pose.x() = msg->pose.pose.orientation.x;
    pose.y() = msg->pose.pose.orientation.y;
    pose.z() = msg->pose.pose.orientation.z;
    pose.w() = msg->pose.pose.orientation.w;
    Eigen::Matrix3d uav_rot = pose.toRotationMatrix();
    for (int i = 0; i < uav_extra_model.points.size(); i++) {
      Eigen::Vector3d model_point;
      model_point(0) = uav_extra_model.points[i].x;
      model_point(1) = uav_extra_model.points[i].y;
      model_point(2) = uav_extra_model.points[i].z;
      model_point = uav_rot * model_point + uav_pos;
      temp_point.x = model_point(0);
      temp_point.y = model_point(1);
      temp_point.z = model_point(2);
      temp_point.intensity =
      ((float)(MAX_INTENSITY - MIN_INTENSITY)) * ((drone_id + 1.0) / (float(drone_num))) +
      MIN_INTENSITY; // set the intensity of the point
      otheruav_points[drone_id].push_back(temp_point);
      otheruav_points_vis.push_back(temp_point);
      uavpt_count++;
    }
  }

  // publish uav point cloud
  // sensor_msgs::PointCloud2 otheruav_points_vis_pcd;
  // pcl::toROSMsg(otheruav_points_vis, otheruav_points_vis_pcd);
  // otheruav_points_vis_pcd.header.frame_id = "world";
  // otheruav_points_vis_pcd.header.stamp = ros::Time::now();
  // pub_uavcloud.publish(otheruav_points_vis_pcd);
}

int comp_time_count = 0;

/// @brief 从话题接入场景地图并准备渲染/针孔所需数据，适用于 use_global_map_topic 分支。
void rcvGlobalPointCloudCallBack(const sensor_msgs::PointCloud2 &pointcloud_map) {
  if (has_global_map)
    return;

  ROS_WARN("Global Pointcloud received for local sensing renderer.");

  pcl::PointCloud<PointType> cloud_input;
  pcl::fromROSMsg(pointcloud_map, cloud_input);
  if (cloud_input.empty()) {
    ROS_WARN("Received empty global pointcloud, waiting for next map.");
    return;
  }

  if (useLidarSensor()) {
    render.read_pointcloud_fromcloud(cloud_input);
  }
  if (!preparePinholeMap(cloud_input)) {
    return;
  }
  has_global_map = true;
}

/// @brief 按 sensor_type 调用针孔投影或 OpenGL 雷达渲染，并发布观测及匹配的传感器位姿。
void renderSensedPoints(const ros::TimerEvent &event) {
  ros::Time t1 = ros::Time::now();

  if (!has_odom || !has_global_map)
    return;

  if (useDepthSensor()) {
    publishCameraPose(odom_);
    publishPinholeDepth(odom_.header.stamp);
    return;
  }

  Eigen::Quaternionf q;
  q.x() = odom_.pose.pose.orientation.x;
  q.y() = odom_.pose.pose.orientation.y;
  q.z() = odom_.pose.pose.orientation.z;
  q.w() = odom_.pose.pose.orientation.w;
  // add pitchW
  double theta = lidar_pitch * M_PI / 180.0;
  Eigen::Matrix3f rot_body2lidar;
  rot_body2lidar << cos(theta), 0, sin(theta), 0, 1, 0, -sin(theta), 0, cos(theta);

  // rotate lidar
  // Eigen::Vector3d eulerAngle_body2lidar(0,0.5236,0);
  // Eigen::AngleAxisd rollAngle(AngleAxisd(eulerAngle_body2lidar(0),Vector3d::UnitX()));
  // Eigen::AngleAxisd pitchAngle(AngleAxisd(eulerAngle_body2lidar(1),Vector3d::UnitY()));
  // Eigen::AngleAxisd yawAngle(AngleAxisd(eulerAngle_body2lidar(2),Vector3d::UnitZ()));
  // rot_body2lidar=yawAngle*pitchAngle*rollAngle;
  // rotate lidar end

  const Eigen::Matrix3f rot(q.toRotationMatrix() * rot_body2lidar);
  Eigen::Quaternionf quat(rot);
  // const Eigen::Matrix3f rot(q.toRotationMatrix());
  const Eigen::Vector3f pos(odom_.pose.pose.position.x, odom_.pose.pose.position.y,
                            odom_.pose.pose.position.z);
  ros::Time time_stamp_ = odom_.header.stamp;

  ros::Time t_pattern = ros::Time::now();
  double time_frominit = (t_pattern - t_init).toSec();

  local_map->points.clear();
  dynamic_input_points.points.clear();

  for (int i = 0; i < drone_num; i++) {
    if (i == drone_id) {
      continue;
    }
    for (int j = 0; j < otheruav_points[i].size(); j++) {
      dynamic_input_points.points.push_back(otheruav_points[i][j]);
    }
    // dynamic_input_points = dynamic_input_points + otheruav_points[i];
  }
  dynamic_input_points = dynamic_input_points + dynobj_points_vis;

  if (dynamic_input_points.points.size() > 0) {
    kdtree_dyn.setInputCloud(dynamic_input_points.makeShared());
  }

  // trans and publish the dynamic point cloud
  sensor_msgs::PointCloud2 dynamic_input_points_pcd;
  pcl::toROSMsg(dynamic_input_points, dynamic_input_points_pcd);
  dynamic_input_points_pcd.header.frame_id = "world";
  dynamic_input_points_pcd.header.stamp = ros::Time::now();
  pub_dyncloud.publish(dynamic_input_points_pcd);

  // dynamic_input_points = otheruav_points_vis + dyn_points_vis;
  // ROS_INFO("dynamic_input_points size = %d", dynamic_input_points.size());
  render.input_dyn_clouds(dynamic_input_points);

  render.render_pointcloud(local_map, pos, quat, time_frominit);

  ros::Time t_afterrender = ros::Time::now();
  double comp_time_temp = (t_afterrender - t1).toSec();
  comp_time_vec.push_back(comp_time_temp);
  if (comp_time_count > 20) {
    comp_time_vec.pop_front();
    geometry_msgs::PoseStamped totaltime_pub;
    totaltime_pub.pose.position.x =
    accumulate(comp_time_vec.begin(), comp_time_vec.end(), 0.0) / comp_time_vec.size();
    comp_time_pub.publish(totaltime_pub);
    // ROS_INFO("Temp compute time = %lf, average compute time = %lf",
    // comp_time_temp,totaltime_pub.pose.position.x);
  } else {
    comp_time_count++;
  }

  local_map->width = local_map->points.size();
  local_map->height = 1;
  local_map->is_dense = true;

  pcl::PointCloud<PointType> local_map_vis;
  local_map_vis.points.reserve(local_map->points.size());
  for (const auto &pt : local_map->points) {
    const Eigen::Vector3f pt_vec(pt.x, pt.y, pt.z);
    if ((pt_vec - pos).norm() >= sensing_horizon - 1.0) {
      continue;
    }
    local_map_vis.points.push_back(pt);
  }
  local_map_vis.width = local_map_vis.points.size();
  local_map_vis.height = 1;
  local_map_vis.is_dense = true;

  // std::cout<< "local map size: " << local_map->points.size() << std::endl;

  pcl::toROSMsg(local_map_vis, local_map_pcd);
  local_map_pcd.header = odom_.header;
  local_map_pcd.header.stamp = time_stamp_;
  local_map_pcd.header.frame_id = "world";
  pub_cloud.publish(local_map_pcd);

  // transform
  std::string sensor_frame_id_ = "sensor";
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transform;
  transform.header.stamp = time_stamp_;
  transform.header.frame_id = "world";
  transform.child_frame_id = sensor_frame_id_;

  transform.transform.translation.x = pos.x();
  transform.transform.translation.y = pos.y();
  transform.transform.translation.z = pos.z();
  transform.transform.rotation.x = q.x();
  transform.transform.rotation.y = q.y();
  transform.transform.rotation.z = q.z();
  transform.transform.rotation.w = q.w();

  br.sendTransform(transform);

  Eigen::Matrix4f sensor2world;
  sensor2world << rot(0, 0), rot(0, 1), rot(0, 2), pos.x(), rot(1, 0), rot(1, 1), rot(1, 2),
  pos.y(), rot(2, 0), rot(2, 1), rot(2, 2), pos.z(), 0, 0, 0, 1;
  Eigen::Matrix4f world2sensor;
  world2sensor = sensor2world.inverse();

  // body frame pointcloud
  point_in_sensor.points.clear();
  // pcl::copyPointCloud(local_map_filled, point_in_sensor);
  pcl::transformPointCloud(*local_map, point_in_sensor, world2sensor);
  point_in_sensor.width = point_in_sensor.points.size();
  point_in_sensor.height = 1;
  point_in_sensor.is_dense = true;

  // pcl::toROSMsg(point_in_sensor, sensor_map_pcd);
  // sensor_map_pcd.header.frame_id = sensor_frame_id_;
  // sensor_map_pcd.header.stamp = time_stamp_;
  // pub_intercloud.publish(sensor_map_pcd);
  publishLidarPose(odom_, time_stamp_, pos, quat);
}

/// @brief 配置渲染器、地图来源、消息和定时器；话题地图与 PCD 参数输入由 use_global_map_topic 区分。
/// 启用 GPU 可执行文件不代表已经切换本节点；主 launch 当前未透传 use_gpu。
int main(int argc, char **argv) {
  ros::init(argc, argv, "pcl_render");
  ros::NodeHandle nh("~");

  // 本段读取启动配置：下方“正常默认”来自当前 launch/XML，实际运行以参数服务器为准。
  // param() 的末项仅是缺参回退；getParam()/必需参数封装没有这一回退，具体见各行说明。
  // 参数读取：观测话题中的机器人名称前缀，默认 quad_0；属于仿真接口命名，不限定机器人实际形态。
  // 配置位置：私有 /pcl_render_node/quadrotor_name；当前主 launch 未覆盖此键，取下面的缺参回退。
  // 缺参回退：std::string("quad_0")；只有参数服务器中没有该键时才使用。
  nh.param("quadrotor_name", quad_name, std::string("quad_0"));
  // 参数读取：写入 /pcl_render_node/sensor_type，匹配同名感知节点的私有句柄。
  // 配置来源：simulator.xml → /pcl_render_node/sensor_type；正常默认启动取 lidar。
  // 缺参回退：std::string("depth")；只有参数服务器中没有该键时才使用。
  nh.param("sensor_type", sensor_type_, std::string("depth"));
  if (!useDepthSensor() && !useLidarSensor()) {
    ROS_ERROR("Unsupported sensor_type '%s'. Expected 'depth' or 'lidar'.", sensor_type_.c_str());
    return 1;
  }
  ROS_INFO("opengl_render_node sensor_type: %s", sensor_type_.c_str());
  // 参数读取：整数开关：是否采用全周雷达视场；影响角度栅格宽度和观测筛选。
  // 配置位置：私有 /pcl_render_node/is_360lidar；当前主 launch 未覆盖此键，取下面的缺参回退。
  // 缺参回退：1；只有参数服务器中没有该键时才使用。
  nh.param("is_360lidar", is_360lidar, 1);
  // 参数读取：模拟观测的最大空间距离，单位 m；不同于 GridMap 的融合射线长度。
  // 配置位置：私有 /pcl_render_node/sensing_horizon；当前主 launch 未覆盖此键，取下面的缺参回退。
  // 缺参回退：40.0；只有参数服务器中没有该键时才使用。
  nh.param("sensing_horizon", sensing_horizon, 40.0);
  // 参数读取：感知渲染和动态物体回调的目标频率，单位 Hz；计算耗时可能使实际频率下降。
  // 配置位置：私有 /pcl_render_node/sensing_rate；当前主 launch 未覆盖此键，取下面的缺参回退。
  // 缺参回退：10.0；只有参数服务器中没有该键时才使用。
  nh.param("sensing_rate", sensing_rate, 10.0);
  // 参数读取：保留的估计频率配置，单位 Hz；当前节点没有据此单独创建定位估计器。
  // 配置位置：私有 /pcl_render_node/estimation_rate；当前主 launch 未覆盖此键，取下面的缺参回退。
  // 缺参回退：10.0；只有参数服务器中没有该键时才使用。
  nh.param("estimation_rate", estimation_rate, 10.0);
  // 参数读取：雷达极坐标角度分辨率，单位度；越小通常意味着更大的角度栅格和计算量。
  // 配置位置：私有 /pcl_render_node/polar_resolution；当前主 launch 未覆盖此键，取下面的缺参回退。
  // 缺参回退：0.2；只有参数服务器中没有该键时才使用。
  nh.param("polar_resolution", polar_resolution, 0.2);
  // 参数读取：雷达水平视场角，单位度；与全周模式共同决定筛选范围。
  // 配置位置：私有 /pcl_render_node/yaw_fov；当前主 launch 未覆盖此键，取下面的缺参回退。
  // 缺参回退：360.0；只有参数服务器中没有该键时才使用。
  nh.param("yaw_fov", yaw_fov, 360.0);
  // 参数读取：雷达垂直视场角，单位度。
  // 配置位置：私有 /pcl_render_node/vertical_fov；当前主 launch 未覆盖此键，取下面的缺参回退。
  // 缺参回退：90.0；只有参数服务器中没有该键时才使用。
  nh.param("vertical_fov", vertical_fov, 90.0);
  // 参数读取：模拟雷达的近端距离配置，单位 m；具体筛除位置见渲染分支。
  // 配置位置：私有 /pcl_render_node/min_raylength；当前主 launch 未覆盖此键，取下面的缺参回退。
  // 缺参回退：1.0；只有参数服务器中没有该键时才使用。
  nh.param("min_raylength", min_raylength, 1.0);
  // 参数读取：场景降采样及部分几何离散使用的长度尺度，单位 m；不是规划器体素分辨率。
  // 配置位置：私有 /pcl_render_node/downsample_res；当前主 launch 未覆盖此键，取下面的缺参回退。
  // 缺参回退：0.1；只有参数服务器中没有该键时才使用。
  nh.param("downsample_res", downsample_res, 0.1);
  // 参数读取：Livox 扫描模式相关配置，仅在相应扫描模式启用时按渲染实现使用。
  // 配置位置：私有 /pcl_render_node/livox_linestep；当前主 launch 未覆盖此键，取下面的缺参回退。
  // 缺参回退：1.4；只有参数服务器中没有该键时才使用。
  nh.param("livox_linestep", livox_linestep, 1.4);
  // 参数读取：Avia 风格扫描模式开关，与其他扫描模式的实际优先级见渲染分支。
  // 配置位置：私有 /pcl_render_node/use_avia_pattern；当前主 launch 未覆盖此键，取下面的缺参回退。
  // 缺参回退：0；只有参数服务器中没有该键时才使用。
  nh.param("use_avia_pattern", use_avia_pattern, 0);
  // 参数读取：场景几何筛选相关配置，沿用渲染实现中的量纲和阈值解释。
  // 配置位置：私有 /pcl_render_node/curvature_limit；当前主 launch 未覆盖此键，取下面的缺参回退。
  // 缺参回退：100.0；只有参数服务器中没有该键时才使用。
  nh.param("curvature_limit", curvature_limit, 100.0);
  // 参数读取：场景空间哈希立方块边长，单位 m，影响视场候选块数量。
  // 配置位置：私有 /pcl_render_node/hash_cubesize；当前主 launch 未覆盖此键，取下面的缺参回退。
  // 缺参回退：5.0；只有参数服务器中没有该键时才使用。
  nh.param("hash_cubesize", hash_cubesize, 5.0);
  // 参数读取：VLP32 风格扫描模式开关，属于模拟采样模式。
  // 配置位置：私有 /pcl_render_node/use_vlp32_pattern；当前主 launch 未覆盖此键，取下面的缺参回退。
  // 缺参回退：0；只有参数服务器中没有该键时才使用。
  nh.param("use_vlp32_pattern", use_vlp32_pattern, 0);
  // 参数读取：MiniCF 风格扫描模式开关；最终模式按后续条件分支确定。
  // 配置位置：私有 /pcl_render_node/use_minicf_pattern；当前主 launch 未覆盖此键，取下面的缺参回退
  // 。
  // 缺参回退：1；只有参数服务器中没有该键时才使用。
  nh.param("use_minicf_pattern", use_minicf_pattern, 1);
  // 参数读取：OS128 风格扫描模式开关。
  // 配置位置：私有 /pcl_render_node/use_os128_pattern；当前主 launch 未覆盖此键，取下面的缺参回退。
  // 缺参回退：0；只有参数服务器中没有该键时才使用。
  nh.param("use_os128_pattern", use_os128_pattern, 0);
  // 参数读取：OS64 风格扫描模式开关。
  // 配置位置：私有 /pcl_render_node/use_os64_pattern；当前主 launch 未覆盖此键，取下面的缺参回退。
  // 缺参回退：0；只有参数服务器中没有该键时才使用。
  nh.param("use_os64_pattern", use_os64_pattern, 0);
  // 参数读取：GPU 雷达安装俯仰角，单位度；渲染前转换为弧度并组合机身旋转。
  // 配置位置：私有 /pcl_render_node/lidar_pitch；当前主 launch 未覆盖此键，取下面的缺参回退。
  // 缺参回退：45.0；只有参数服务器中没有该键时才使用。
  nh.param("lidar_pitch", lidar_pitch, 45.0);
  // 参数读取：合成深度图宽度，单位 px，须与主点/焦距配置配套。
  // 配置位置：私有 /pcl_render_node/cam_width；当前主 launch 未覆盖此键，取下面的缺参回退。
  // 缺参回退：cam_width 的已有初值（640 px）；主 launch 未额外覆盖此相机键。
  nh.param("cam_width", cam_width, cam_width);
  // 参数读取：合成深度图高度，单位 px。
  // 配置位置：私有 /pcl_render_node/cam_height；当前主 launch 未覆盖此键，取下面的缺参回退。
  // 缺参回退：cam_height 的已有初值（480 px）；主 launch 未额外覆盖此相机键。
  nh.param("cam_height", cam_height, cam_height);
  // 参数读取：针孔水平焦距，单位 px，必须为正。
  // 配置位置：私有 /pcl_render_node/cam_fx；当前主 launch 未覆盖此键，取下面的缺参回退。
  // 缺参回退：cam_fx 的已有初值（约 387.229 px）；主 launch 未额外覆盖此相机键。
  nh.param("cam_fx", cam_fx, cam_fx);
  // 参数读取：针孔垂直焦距，单位 px，必须为正。
  // 配置位置：私有 /pcl_render_node/cam_fy；当前主 launch 未覆盖此键，取下面的缺参回退。
  // 缺参回退：cam_fy 的已有初值（约 387.229 px）；主 launch 未额外覆盖此相机键。
  nh.param("cam_fy", cam_fy, cam_fy);
  // 参数读取：针孔主点水平坐标，单位 px。
  // 配置位置：私有 /pcl_render_node/cam_cx；当前主 launch 未覆盖此键，取下面的缺参回退。
  // 缺参回退：cam_cx 的已有初值（约 321.046 px）；主 launch 未额外覆盖此相机键。
  nh.param("cam_cx", cam_cx, cam_cx);
  // 参数读取：针孔主点垂直坐标，单位 px。
  // 配置位置：私有 /pcl_render_node/cam_cy；当前主 launch 未覆盖此键，取下面的缺参回退。
  // 缺参回退：cam_cy 的已有初值（约 243.450 px）；主 launch 未额外覆盖此相机键。
  nh.param("cam_cy", cam_cy, cam_cy);

  // 参数读取：GPU 渲染后处理配置开关，具体效果由渲染器实现决定。
  // 配置位置：私有 /pcl_render_node/use_gaussian_filter；当前主 launch 未覆盖此键，取下面的缺参回退
  // 。
  // 缺参回退：0；只有参数服务器中没有该键时才使用。
  nh.param("use_gaussian_filter", use_gaussian_filter, 0);

  // dyn parameters
  // 参数读取：是否启用动态障碍仿真；默认关闭，启用后由动态回调更新场景点云。
  // 配置位置：私有 /pcl_render_node/dynobj_enable；当前主 launch 未覆盖此键，取下面的缺参回退。
  // 缺参回退：0；只有参数服务器中没有该键时才使用。
  nh.param("dynobj_enable", dynobj_enable, 0);
  // 参数读取：动态障碍的几何尺寸尺度，单位 m，具体解释随障碍类型变化。
  // 配置位置：私有 /pcl_render_node/dynobject_size；当前主 launch 未覆盖此键，取下面的缺参回退。
  // 缺参回退：0.0；只有参数服务器中没有该键时才使用。
  nh.param("dynobject_size", dynobject_size, 0.0);
  // 参数读取：动态障碍数量；仅启用动态障碍分支后参与初始化。
  // 配置位置：私有 /pcl_render_node/dynobject_num；当前主 launch 未覆盖此键，取下面的缺参回退。
  // 缺参回退：0；只有参数服务器中没有该键时才使用。
  nh.param("dynobject_num", dynobject_num, 0);
  // 参数读取：动态障碍运动模式配置，实际运动分支见 dynobjGenerate()。
  // 配置位置：私有 /pcl_render_node/dyn_mode；当前主 launch 未覆盖此键，取下面的缺参回退。
  // 缺参回退：0；只有参数服务器中没有该键时才使用。
  nh.param("dyn_mode", dyn_mode, 0);
  // 参数读取：动态障碍速度尺度，单位 m/s。
  // 配置位置：私有 /pcl_render_node/dyn_velocity；当前主 launch 未覆盖此键，取下面的缺参回退。
  // 缺参回退：0.0；只有参数服务器中没有该键时才使用。
  nh.param("dyn_velocity", dyn_velocity, 0.0);

  // 参数读取：是否加载额外机身点云模型；保留上游多机仿真能力，默认主链路不依赖它。
  // 配置位置：私有 /pcl_render_node/use_uav_extra_model；当前主 launch 未覆盖此键，取下面的缺参回退
  // 。
  // 缺参回退：0；只有参数服务器中没有该键时才使用。
  nh.param("use_uav_extra_model", use_uav_extra_model, 0);

  // 参数读取：是否启用仿真器独立的几何碰撞检查，默认关闭；与规划器安全检查不是同一个开关。
  // 配置位置：私有 /pcl_render_node/collisioncheck_enable；当前主 launch 未覆盖此键，取下面的缺参回
  // 退。
  // 缺参回退：0；只有参数服务器中没有该键时才使用。
  nh.param("collisioncheck_enable", collisioncheck_enable, 0);
  // 参数读取：仿真器独立碰撞检查的距离尺度，单位 m，不等同于规划器双圆柱模型。
  // 配置位置：私有 /pcl_render_node/collision_range；当前主 launch 未覆盖此键，取下面的缺参回退。
  // 缺参回退：0.3；只有参数服务器中没有该键时才使用。
  nh.param("collision_range", collision_range, 0.3);

  // 参数读取：调试点云输出配置，保留上游文件输出分支。
  // 配置位置：私有 /pcl_render_node/output_pcd；当前主 launch 未覆盖此键，取下面的缺参回退。
  // 缺参回退：0；只有参数服务器中没有该键时才使用。
  nh.param("output_pcd", output_pcd, 0);
  // 参数读取：GPU 分支：随机地图时从话题接收场景，PCD 模式下按程序参数加载文件。
  // 配置来源：simulator.xml → /pcl_render_node/use_global_map_topic；该GPU 分支启用时取 true。
  // 缺参回退：false；只有参数服务器中没有该键时才使用。
  nh.param("use_global_map_topic", use_global_map_topic, false);

  // 参数读取：感知节点的场景 x 尺寸参数，单位 m，不是 GridMap 的局部窗口大小。
  // 配置来源：simulator.xml → /pcl_render_node/map/x_size；正常默认启动取 40.0。
  // 无缺省实参；当前代码直接调用 getParam()，返回结果应结合本分支的上游配置理解。
  nh.getParam("map/x_size", x_size);
  // 参数读取：感知节点的场景 y 尺寸参数，单位 m。
  // 配置来源：simulator.xml → /pcl_render_node/map/y_size；正常默认启动取 40.0。
  // 无缺省实参；当前代码直接调用 getParam()，返回结果应结合本分支的上游配置理解。
  nh.getParam("map/y_size", y_size);
  // 参数读取：感知节点的场景 z 尺寸参数，单位 m。
  // 配置来源：simulator.xml → /pcl_render_node/map/z_size；正常默认启动取 5.0。
  // 无缺省实参；当前代码直接调用 getParam()，返回结果应结合本分支的上游配置理解。
  nh.getParam("map/z_size", z_size);

  // subscribe other uav pos
  // 参数读取：多机仿真的机器人数量，默认 1。
  // 配置位置：私有 /pcl_render_node/uav_num；当前主 launch 未覆盖此键，取下面的缺参回退。
  // 缺参回退：1；只有参数服务器中没有该键时才使用。
  nh.param("uav_num", drone_num, 1);
  // 参数读取：本机在多机列表中的编号；相关订阅跳过自身索引。
  // 配置位置：私有 /pcl_render_node/drone_id；当前主 launch 未覆盖此键，取下面的缺参回退。
  // 缺参回退：0；只有参数服务器中没有该键时才使用。
  nh.param("drone_id", drone_id, 0);
  other_uav_pos.resize(drone_num);
  other_uav_rcv_time.resize(drone_num);
  otheruav_points.resize(drone_num);
  otheruav_pointsindex.resize(drone_num);
  ros::Subscriber *subs = new ros::Subscriber[drone_num];
  for (int i = 0; i < drone_num; i++) {
    if (i == drone_id) {
      continue;
    }
    string topic = "/quad_";
    topic += to_string(i);
    topic += "/body_pose";
    cout << topic << endl;
    subs[i] =
    // 运行连接（接收入口）：可选其他机器人 Odometry，循环按 /quad_i/body_pose 注册并跳过本机，供多
    // 机几何显示/模拟遮挡。
    // 队列容量：1000 条消息，不是发布/规划频率。 到达后交给 boost::bind(&multiOdometryCallback, _1,
    // i)。
    nh.subscribe<nav_msgs::Odometry>(topic, 1000, boost::bind(&multiOdometryCallback, _1, i));
  }

  pcl::VoxelGrid<PointType> sor;
  int pcd_read_status;
  if (use_uav_extra_model) {
    string uav_model_path;
    uav_model_path = ros::package::getPath(
    "odom_visualization"); //=
                           //"/home/mars/catkin_ws2/src/Exploration_sim/octomap_mapping/octomap_server"
    uav_model_path.append("/meshes/yunque001.pcd");
    std::cout << "\nFound pkg_path = " << uav_model_path << std::endl;

    pcd_read_status = pcl::io::loadPCDFile<PointType>(uav_model_path, uav_extra_model);
    if (pcd_read_status == -1) {
      cout << "can't read uav extra model file." << endl;
      return 0;
    }
    // downsample uav model
    sor.setInputCloud(uav_extra_model.makeShared());
    sor.setLeafSize(downsample_res, downsample_res, downsample_res);
    sor.filter(uav_extra_model);
  }

  uav_size[0] = 0.4;
  uav_size[1] = 0.4;
  uav_size[2] = 0.3;
  drone_drawpoints_num[0] = (uav_size[0] / downsample_res);
  drone_drawpoints_num[1] = (uav_size[1] / downsample_res);
  drone_drawpoints_num[2] = (uav_size[2] / downsample_res);
  uav_points_num = drone_drawpoints_num[0] * drone_drawpoints_num[1] * drone_drawpoints_num[2];
  ROS_INFO("drone_drawpoints_num = %d,%d,%d, Uav points num = %d", drone_drawpoints_num[0],
           drone_drawpoints_num[1], drone_drawpoints_num[2], uav_points_num);

  // render.setParameters(400,400,250,250,downsample_res,0.1,sensing_horizon,sensing_rate,use_avia_pattern);
  int image_width;
  int image_height;
  if (is_360lidar) {
    image_width = ceil(360.0 / polar_resolution);
  } else {
    image_width = ceil(yaw_fov / polar_resolution);
  }
  image_height = ceil(vertical_fov / polar_resolution);
  render.setParameters(image_width, image_height, 250, 250, downsample_res, polar_resolution,
                       yaw_fov, vertical_fov, 0.1, sensing_horizon, sensing_rate, use_avia_pattern,
                       use_os128_pattern, use_minicf_pattern);

  if (use_global_map_topic) {
    // 运行连接（接收入口）：感知器私有全局场景输入，经 remap 接 /map_generator/global_cloud；只有场
    // 景和机身状态都就绪才渲染。
    // 队列容量：1 条消息，不是发布/规划频率。 到达后交给 rcvGlobalPointCloudCallBack。
    global_map_sub = nh.subscribe("global_map", 1, rcvGlobalPointCloudCallBack);
  } else {
    if (argc < 2) {
      ROS_ERROR("opengl_render_node requires a PCD file argument unless ~use_global_map_topic is true.");
      return 1;
    }
    file_name = argv[1];
    if (useLidarSensor()) {
      render.read_pointcloud_fromfile(file_name);
    }
    if (!loadPinholeMapFromFile(file_name)) {
      return 1;
    }
    has_global_map = true;
  }
  // home/dji/kong_ws/src/Exploration_sim/uav_simulator/map_generator/resource/Knowles_merge_01cutoff.pcd
  // /home/dji/meshmap/Knowles_local_sor_001.pcd

  if (collisioncheck_enable) {
    if (cloud_all_map.empty()) {
      if (file_name.empty() || !loadPinholeMapFromFile(file_name)) {
        cout << "can't read global" << endl;
        return 0;
      }
    }

    if (cloud_all_map.empty()) {
      ROS_ERROR("Global map is empty.");
      return 0;
    }

    ROS_INFO("Global map and kdtree ready.");

    if (use_uav_extra_model) {
      // get max and min xyz of uav model
      pcl::getMinMax3D(uav_extra_model, uav_modelmin, uav_modelmax);
      uavhash_xsize = ceil((uav_modelmax.x - uav_modelmin.x) / downsample_res);
      uavhash_ysize = ceil((uav_modelmax.y - uav_modelmin.y) / downsample_res);
      uavhash_zsize = ceil((uav_modelmax.z - uav_modelmin.z) / downsample_res);

      // input the uav model point clouds into voxel grid hash map
      for (int i = 0; i < uav_extra_model.points.size(); i++) {
        PointType p = uav_extra_model.points[i];
        int x = floor((p.x - uav_modelmin.x) / downsample_res);
        int y = floor((p.y - uav_modelmin.y) / downsample_res);
        int z = floor((p.z - uav_modelmin.z) / downsample_res);
        long int hash = x + y * uavhash_xsize + z * uavhash_xsize * uavhash_ysize;
        uavpoint_hashmap[hash] = true;
      }

      ROS_INFO("UAV model read and input hash map done. UAV model size = %d,%d,%d", uavhash_xsize,
               uavhash_ysize, uavhash_zsize);
    }
  }

  if (dynobj_enable == 1) {
    if (!collisioncheck_enable && cloud_all_map.empty()) {
      if (file_name.empty() || !loadPinholeMapFromFile(file_name)) {
        cout << "can't read global" << endl;
        return 0;
      }
    }

    map_min(0) = global_mapmin.x;
    map_min(1) = global_mapmin.y;
    map_min(2) = global_mapmin.z;
    map_max(0) = global_mapmax.x;
    map_max(1) = global_mapmax.y;
    map_max(2) = global_mapmax.z;

    // ROS_INFO("Global map min x = %f, max x = %f" , global_mapmin.x , global_mapmax.x);

    // dynamic objects initial pos generate
    srand((unsigned)time(NULL));
    for (int i = 0; i < dynobject_num; i++) {
      Eigen::Vector3d dyntemp_pos;
      dyntemp_pos(0) =
      rand() / double(RAND_MAX) * (global_mapmax.x - global_mapmin.x) + global_mapmin.x;
      dyntemp_pos(1) =
      rand() / double(RAND_MAX) * (global_mapmax.y - global_mapmin.y) + global_mapmin.y;
      dyntemp_pos(2) =
      rand() / double(RAND_MAX) * (global_mapmax.z - global_mapmin.z) + global_mapmin.z;
      dynobj_poss.push_back(dyntemp_pos);

      // random generate dynamic object type and motion mode
      int dynobj_type = rand() % 3;
      dynobj_types.push_back(dynobj_type);
      // int dynobj_motion_mode = rand() % 2;
      dynobj_move_modes.push_back(dynobj_type);

      Eigen::Vector3d dyntemp_dir, dyntemp_dir_polar;
      dyntemp_dir_polar(0) = rand() / double(RAND_MAX) * 3.1415926;
      dyntemp_dir_polar(1) = (rand() / double(RAND_MAX) - 0.5) * 3.1415926;
      dyntemp_dir[2] = dyn_velocity * sin(dyntemp_dir_polar(1));
      dyntemp_dir[1] = dyn_velocity * cos(dyntemp_dir_polar(1)) * sin(dyntemp_dir_polar(0));
      dyntemp_dir[0] = dyn_velocity * cos(dyntemp_dir_polar(1)) * cos(dyntemp_dir_polar(0));
      dynobj_dir.push_back(dyntemp_dir);

      if (dyn_obs_diff_size_on) {
        double dynobject_rand_size =
        rand() / double(RAND_MAX) * (dynobject_size - downsample_res) + downsample_res;
        dyn_obs_size_vec.push_back(dynobject_rand_size);
      }

      // dyn_start_time = ros::Time::now();
      dyn_start_time_vec.push_back(ros::Time::now());
    }
  }

  // subscribe point cloud
  std::string body_pose_topic;
  // 参数读取：机身 Odometry 接口；默认仿真 /quad_0/body_pose，真机由父 launch 切到
  // /LIO/odom_vehicle。
  // 配置来源：run.launch → /body_pose_topic；正常默认启动取 /quad_0/body_pose。
  // 缺参回退：std::string("/quad_0/body_pose")；只有参数服务器中没有该键时才使用。
  ros::param::param<std::string>("/body_pose_topic", body_pose_topic, std::string("/quad_0/body_pose"));
  // 运行连接（接收入口）：机身 Odometry 接口，实际名称由 /body_pose_topic 指定；默认仿真
  // /quad_0/body_pose，接入规划/控制/感知/显示相应回调。
  // 队列容量：50 条消息，不是发布/规划频率。 到达后交给 rcvOdometryCallback。
  odom_sub = nh.subscribe(body_pose_topic, 50, rcvOdometryCallback);

  // publishers
  //   pub_dyncloud = nh.advertise<sensor_msgs::PointCloud2>("/pcl_render_node/dyn_cloud", 10);
  // pub_intercloud = nh.advertise<sensor_msgs::PointCloud2>("/pcl_render_node/explored_cloud", 10);
  // 运行连接（输出入口）：感知器可选动态物体显示点云，属于模拟场景调试接口。
  // 队列容量：10 条消息，不是发布/规划频率。
  pub_dyncloud = nh.advertise<sensor_msgs::PointCloud2>("dyn_cloud", 10);
  if (useDepthSensor()) {
    // 运行连接（输出入口）：模拟相机位姿输出，默认 /quad_0/camera_pose，供 depth 分支同步融合。
    // 队列容量：10 条消息，不是发布/规划频率。
    camera_pose_pub_ = nh.advertise<nav_msgs::Odometry>(quadTopic("camera_pose"), 10);
    // 运行连接（输出入口）：模拟深度输出，默认 /pcl_render_node/depth，米制 32FC1，供 depth 模式建
    // 图。
    // 队列容量：10 条消息，不是发布/规划频率。
    pinhole_depth_pub_ = nh.advertise<sensor_msgs::Image>("depth", 10);
  } else {
    // 运行连接（输出入口）：模拟传感器坐标下的点云调试输出，区别于默认规划输入的世界系 cloud。
    // 队列容量：10 条消息，不是发布/规划频率。
    pub_intercloud = nh.advertise<sensor_msgs::PointCloud2>("sensor_cloud", 10);
    // 运行连接（输出入口）：模拟雷达的世界系观测输出，默认 /pcl_render_node/cloud，供规划器 lidar
    // 建图。
    // 队列容量：10 条消息，不是发布/规划频率。
    pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("cloud", 10);
    // 运行连接（输出入口）：模拟雷达位姿输出，默认 /quad_0/lidar_pose，与激光观测使用的姿态对应。
    // 队列容量：10 条消息，不是发布/规划频率。
    lidar_pose_pub_ = nh.advertise<nav_msgs::Odometry>(quadTopic("lidar_pose"), 10);
  }
  // 运行连接（输出入口）：感知器可选额外机身/多机几何显示接口，沿用上游仿真命名。
  // 队列容量：10 条消息，不是发布/规划频率。
  pub_uavcloud = nh.advertise<sensor_msgs::PointCloud2>("uav_cloud", 10);
  // 运行连接（输出入口）：感知计算耗时的调试输出，借用 PoseStamped 承载统计值，不是机器人目标位姿。
  // 队列容量：10 条消息，不是发布/规划频率。
  comp_time_pub = nh.advertise<geometry_msgs::PoseStamped>("simulator_compute_time", 10);
  // 运行连接（输出入口）：GPU 感知分支的独立碰撞检查调试点云，不取代规划器安全回调。
  // 队列容量：10 条消息，不是发布/规划频率。
  pub_collisioncloud = nh.advertise<sensor_msgs::PointCloud2>("collision_cloud", 10);
  double sensing_duration = 1.0 / sensing_rate;

  t_init = ros::Time::now();

  // 定时任务：按 sensing_rate 生成模拟观测，缺省 10 Hz；与机身状态订阅频率分开。
  local_sensing_timer = nh.createTimer(ros::Duration(sensing_duration), renderSensedPoints);
  // 定时任务：按 sensing_rate 更新可选动态场景，缺省 10 Hz；dynobj_enable 未启用时不推进该场景。
  dynobj_timer = nh.createTimer(ros::Duration(sensing_duration), dynobjGenerate);

  sensor2body << 0.0, 0.0, 1.0, 0.0,
      -1.0, 0.0, 0.0, 0.0,
      0.0, -1.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 1.0;

  ros::Rate rate(100);
  bool status = ros::ok();
  // rate.sleep();

  // // use asyncspinner
  // ros::AsyncSpinner spinner(3);
  // spinner.start();

  // // use multithread spinner
  // ros::MultiThreadedSpinner spinner(4);
  // spinner.spin();

  while (status) {
    ros::spinOnce();
    status = ros::ok();
    rate.sleep();
  }

  // ros::spin();

  return 0;
}
