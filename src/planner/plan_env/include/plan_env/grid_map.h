/// @file
/// 滑动三维地图的状态、ROS 接口及高频内联查询。
/// 全局体素索引与复用内存地址不同；XYZ 查询先验证窗口范围，再映射到取模地址。
/// 占用值为 log-odds，膨胀计数记录多个占用源对同一单元的重叠贡献。
#ifndef _GRID_MAP_H
#define _GRID_MAP_H

#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <algorithm>
#include <cv_bridge/cv_bridge.h>
#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <random>
#include <nav_msgs/Odometry.h>
#include <queue>
#include <ros/ros.h>
#include <tuple>
#include <visualization_msgs/Marker.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>

#include <plan_env/raycast.h>

#define logit(x) (log((x) / (1 - (x))))

using namespace std;
namespace tf {
class TransformBroadcaster;
}

// 通用矩阵哈希工具；GridMap 主缓冲的地址映射仍使用全局体素索引取模。
template <typename T>
struct matrix_hash : std::unary_function<T, size_t> {
  std::size_t operator()(T const& matrix) const {
    size_t seed = 0;
    for (size_t i = 0; i < matrix.size(); ++i) {
      auto elem = *(matrix.data() + i);
      seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
  }
};

// constant parameters

/// @brief 地图尺寸/分辨率、概率模型、传感器内参与外参等配置；长度单位 m。
struct MappingParameters {

  /* map properties */
  // map_origin_：当前逻辑窗口最小角（世界系 m），滑动后更新；map_size_：配置窗口尺寸，默认 (10,10,5) m。
  Eigen::Vector3d map_origin_, map_size_;
  // 当前窗口世界坐标边界，由体素边界乘分辨率得到，isInMap() 用它判断位置是否可查询。
  Eigen::Vector3d map_min_boundary_, map_max_boundary_;  // map range in pos
  // 窗口各轴体素数，ceil(size/resolution)，默认 (200,200,100)，对应固定缓冲区 4,000,000 单元。
  Eigen::Vector3i map_voxel_num_;                        // map range in index
  // 窗口内全局体素索引的最小/最大角；滑动后改变，区别于数组取模后的本地下标。
  Eigen::Vector3i map_bound_min_idx_, map_bound_max_idx_;
  // 窗口中心对应的全局体素索引；与新传感器中心索引比较，决定是否滑窗。
  Eigen::Vector3i map_origin_idx_;
  // 单轮融合范围的各轴半长，未单独配置时为窗口尺寸的一半，默认 (5,5,2.5) m。
  Eigen::Vector3d local_update_range_;
  // resolution_：grid_map/resolution 默认 0.05 m；resolution_inv_：倒数，默认 20，用于位置/索引换算。
  double resolution_, resolution_inv_;
  // grid_map/obstacles_inflation_z_up/down，默认各 0.1 m；构造相对障碍体素的上下膨胀偏移。
  double obstacles_inflation_z_up, obstacles_inflation_z_down;
  // grid_map/double_cylinder_radius/offset，默认 0.25/0.18 m；前者生成水平膨胀，后者定位前后查询中心。
  double double_cylinder_radius_, double_cylinder_offset_;
  // grid_map/map_sliding_en，缺省 true；false 时 updateSlidingMap() 直接返回。
  bool map_sliding_en_;
  // grid_map/map_sliding_thresh，正常配置 0.2 m；按分辨率换算为索引位移阈值。
  double map_sliding_thresh_;
  // ceil(map_sliding_thresh_/resolution_)，至少为 1；默认 4 格，某轴中心索引差达到阈值即触发滑动。
  int map_sliding_thresh_vox_;
  // grid_map/frame_id 与 sliding_map_frame_id，缺省 world/sliding_map；分别用于地图输出和机身随动显示 TF。
  string frame_id_, sliding_map_frame_id_;

  /* depth camera intrinsics */
  // 深度图主点与焦距，单位 px，来自父 launch 所选标定；默认仿真约为 (321.046,243.450,387.229,387.229)。
  double cx_, cy_, fx_, fy_;

  /* depth image projection filtering */
  // grid_map/depth_filter_maxdist/mindist，默认 3.0/0.3 m；过近跳过，过远转为超量程自由空间射线。
  double depth_filter_maxdist_, depth_filter_mindist_;
  // grid_map/depth_filter_margin 默认 1 px，反投影时跳过四周边缘。
  int depth_filter_margin_;
  // grid_map/k_depth_scaling_factor 默认 1000，raw_depth/该值得到米制深度；32FC1 输入先按该因子量化。
  double k_depth_scaling_factor_;
  // grid_map/skip_pixel 默认 2，两图像轴均按该步长抽样；投影点缓冲区大小也据此预分配。
  int skip_pixel_;

  /* raycasting */
  // 占用融合参数，默认依次为 0.85、0.30、0.12、0.98、0.80；分别控制 hit、miss、截断下限、上限和占用判定。
  double p_hit_, p_miss_, p_min_, p_max_, p_occ_;  // occupancy probability
  // 上述概率经 logit 转换后的更新量和阈值；默认约为 +1.735、-0.847、-1.992、+3.892、+1.386。
  // raycastProcess() 累加正/负证据并截断，applyOccupancyUpdate() 根据占用跨阈值更新膨胀层。
  double prob_hit_log_, prob_miss_log_, clamp_min_log_, clamp_max_log_,
      min_occupancy_log_;                   // logit of occupancy probability
  // max_ray_length_：grid_map/max_ray_length，默认 5.0 m，限制占用融合射线长度。
  // min_ray_length_：当前未读取或使用的保留字段，不是模拟雷达的 min_raylength 参数。
  double min_ray_length_, max_ray_length_;  // range of doing raycasting

  /* visualization and computation time display */
  // vis_height_：默认 0.3 m，显示时裁掉高于传感器 z+该值的点，不改占用数据。
  // ground_height_：缺省 0 m，用于初始窗口 z 原点和更新范围处理，不是当前机身高度测量。
  double vis_height_, ground_height_;
  // grid_map/show_occ_time，缺省 false；当前耗时打印代码已注释，设为 true 也不会恢复那段统计。
  bool show_occ_time_;

  /* mapping sensor input */
  // grid_map/sensor_type，默认启动传入 lidar；决定点云还是深度+位姿同步输入，无效值回退 lidar。
  string sensor_type_;
  // 仿真默认 true，真机配置 false；决定点云 XYZ 可直接使用还是需要按传感器位姿变到世界系。
  bool cloud_is_world_;
  // 由 is_real_world 配置传入；true 时输入位姿再乘下方对应安装外参，仿真已经给传感器位姿时为 false。
  bool need_extrinsic_;
  // initMap() 固定的雷达外参，平移约 (-0.011,-0.02329,0.04412) m，旋转为单位矩阵。
  Eigen::Matrix4d lidar_extrinsic_;
  // initMap() 固定的深度相机安装旋转/平移矩阵，仅 need_extrinsic=true 的深度位姿处理使用。
  Eigen::Matrix4d depth_extrinsic_;

  /* active mapping */
  // initMap() 置 0.01，用于让初始 log-odds 比 clamp_min_log_ 更低以标记未知；不是概率或距离。
  double unknown_flag_;
};

// intermediate mapping data for fusion

/// @brief 地图缓冲区、观测缓存与融合状态；更新这些数据必须保持占用层与膨胀层的一致性。
struct MappingData {
  // 每体素占用 log-odds、膨胀标志与覆盖计数；当前这里不存储 ESDF 距离场。

  // 每体素的占用 log-odds；初始为 clamp_min_log_-unknown_flag_，有观测后按 hit/miss 融合。
  std::vector<double> occupancy_buffer_;
  // 膨胀后的碰撞标志，初值全 0；供 getInflateOccupancy() 查询，不能直接解释为原始测量概率。
  std::vector<char> occupancy_buffer_inflate_;
  // 每格被多少个占用源膨胀覆盖，初值全 0；防止删除一个源时误清除其他源的覆盖。
  std::vector<int> occupancy_buffer_inflate_cnt_;
  // 由圆柱半径和上下裕量预计算的整数偏移集合；占用源跨阈值时复用以增减计数。
  vector<Eigen::Vector3i> inflate_offsets_;

  // raycast origin and sensor pose data

  // 最近有效传感器位姿的世界位置（m），初值零；有 has_ray_pose_ 后才用于射线原点。
  Eigen::Vector3d ray_pos_;
  // 传感器到世界系的旋转，初值单位四元数；位姿回调归一化并按需乘安装外参后更新。
  Eigen::Quaterniond ray_q_;
  // 机身里程计位置缓存，初值零；只提供 world→sliding_map 的显示平移，实际窗口滑动以 ray_pos_ 为中心。
  Eigen::Vector3d sliding_map_frame_pos_;

  // depth image data

  // 最新同步深度图，内部按 16UC1 使用；32FC1 输入转换后保存，投影时再除比例因子得到 m。
  cv::Mat depth_image_;
  // 保留图像计数字段，初始化为 0；当前主路径没有递增或消费。
  int image_cnt_;
  // flags of map state

  // 初值 false；观测准备好后置 true，更新定时器融合完成后清 false，避免无新数据时重复融合。
  bool occ_need_update_;
  // 初值 false；true 表示本轮 proj_points_ 已由点云准备，false 时更新回调先反投影深度。
  bool use_cloud_update_;
  // 初值 false；projectDepthImage() 首次进入后置 true，并跳过该次投影。
  bool has_first_depth_;
  // 初值均 false；分别表示已经收到可用传感器位姿和点云，不是定位或消息新鲜度的综合健康指标。
  bool has_ray_pose_, has_cloud_;

  // depth image projected point cloud

  // 本次待融合的世界观测点缓冲，来自点云或深度反投影；只处理前 proj_points_cnt 个有效条目。
  vector<Eigen::Vector3d> proj_points_;
  // 当前有效观测点数，初始化为 0，准备每轮观测时重新计数；不等于缓冲区容量。
  int proj_points_cnt;

  // flag buffers for speeding up raycasting

  // 本轮每格 hit 次数及 hit+miss 次数，初值 0；融合时用多数证据选择正/负更新，随后清零。
  vector<short> count_hit_, count_hit_and_miss_;
  // 射线穿越/端点的本轮去重标记，初始 -1；用 raycast_num_ 标识当前轮次。
  vector<char> flag_traverse_, flag_rayend_;
  // 当前射线融合轮次标记，初值 0，每轮更新；不是传感器消息序号。
  char raycast_num_;
  // 本轮至少出现过一次证据的体素索引队列；融合时逐个弹出并消费计数。
  queue<Eigen::Vector3i> cache_voxel_;

  // range of updating grid

  // 最近有效融合区域的体素边界，初值为当前窗口范围；用于后续区域显示和范围处理。
  Eigen::Vector3i local_bound_min_, local_bound_max_;

  // computation time

  // 保留的总耗时/峰值缓存，初值 0；当前更新与打印已注释，不是正在维护的性能指标。
  double fuse_time_, max_fuse_time_;
  // 初值 0，深度位姿回调接受窗口内观测时递增；不是所有雷达融合次数的统一统计。
  int update_num_;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/// @brief 位姿和观测接收、射线融合、窗口滑动、碰撞查询及显示发布的统一对象。
class GridMap {
public:
  GridMap() {}
  ~GridMap() {}

  enum { INVALID_IDX = -10000 };

  // occupancy map management
  void resetBuffer();
  void resetBuffer(Eigen::Vector3d min, Eigen::Vector3d max);

  inline void posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id);
  inline void indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos);
  inline int toAddress(const Eigen::Vector3i& id);
  inline int toAddress(int& x, int& y, int& z);
  inline bool isInMap(const Eigen::Vector3d& pos);
  inline bool isInMap(const Eigen::Vector3i& idx);

  inline void setOccupancy(Eigen::Vector3d pos, double occ = 1);
  inline void setOccupied(Eigen::Vector3d pos);
  inline int getOccupancy(Eigen::Vector3d pos);
  inline int getOccupancy(Eigen::Vector3i id);
  inline int getInflateOccupancy(Eigen::Vector3d pos, double yaw);

  inline void boundIndex(Eigen::Vector3i& id);
  inline bool isUnknown(const Eigen::Vector3i& id);
  inline bool isUnknown(const Eigen::Vector3d& pos);
  inline bool isKnownFree(const Eigen::Vector3i& id);
  inline bool isKnownOccupied(const Eigen::Vector3i& id);

  void initMap(ros::NodeHandle& nh);

  void publishMap();
  void publishMapInflate(bool all_info = false);

  void publishUnknown();
  void publishDepth();
  void publishDepthCloud();
  void publishSlidingMapBBox();
  void publishSlidingMapFrame();

  bool hasDepthObservation();
  bool odomValid();
  void getRegion(Eigen::Vector3d& ori, Eigen::Vector3d& size);
  inline double getResolution();
  Eigen::Vector3d getOrigin();
  int getVoxelNum();

  typedef std::shared_ptr<GridMap> Ptr;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  // 本实例的地图配置和派生几何缓存，由 initMap() 建立、滑动逻辑更新；字段语义见本文件定义。
  MappingParameters mp_;
  // 本实例的观测、占用、膨胀和射线工作数据，由输入回调与地图更新回调维护。
  MappingData md_;

  // get depth image and sensor pose
  void depthPoseCallback(const sensor_msgs::ImageConstPtr& img,
                         const nav_msgs::OdometryConstPtr& pose);
  void sensorPoseCallback(const nav_msgs::OdometryConstPtr& pose);
  void slidingMapFrameCallback(const nav_msgs::OdometryConstPtr& pose);
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& img);

  // update occupancy by raycasting
  void updateOccupancyCallback(const ros::TimerEvent& /*event*/);
  void visCallback(const ros::TimerEvent& /*event*/);

  // main update process
  void projectDepthImage();
  void raycastProcess();

  inline void inflatePoint(const Eigen::Vector3i& pt, int inf_step_xy, int inf_step_z_up, int inf_step_z_down, vector<Eigen::Vector3i>& pts);
  inline int getInflateOccupancyFromBuffer(Eigen::Vector3d pos, const std::vector<char>& buffer);
  inline int getLocalIndex(int id, int dim) const;
  inline int toAddressLocal(const Eigen::Vector3i& id_l) const;
  inline int toAddressLocal(int x, int y, int z) const;
  int setCacheOccupancy(Eigen::Vector3d pos, int occ);
  Eigen::Vector3d closetPointInMap(const Eigen::Vector3d& pt, const Eigen::Vector3d& ray_pos);
  void updateSlidingMap(const Eigen::Vector3d& center);
  void updateMapBoundaryFromIndex();
  void resetAllMapData();
  void resetCellByAddress(int addr);
  void resetCellByAddressForSliding(int addr, const std::vector<char>& clear_mask);
  void hashIdToGlobalIndex(int addr, Eigen::Vector3i& id_g) const;
  void applyOccupancyUpdate(const Eigen::Vector3i& id, double new_log_odds);
  void rebuildInflationOffsets();
  void updateInflation(const Eigen::Vector3i& id, int delta, const std::vector<char>* ignore_mask = nullptr);
  void updateInflationLayer(const Eigen::Vector3i& id, int delta,
                            const vector<Eigen::Vector3i>& offsets,
                            std::vector<int>& cnt_buffer,
                            std::vector<char>& flag_buffer,
                            const std::vector<char>* ignore_mask);

  // typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image,
  // nav_msgs::Odometry> SyncPolicyImageOdom; typedef
  // message_filters::sync_policies::ExactTime<sensor_msgs::Image,
  // nav_msgs::Odometry> SyncPolicyImagePose;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, nav_msgs::Odometry>
      SyncPolicyImagePose;
  typedef shared_ptr<message_filters::Synchronizer<SyncPolicyImagePose>> SynchronizerImagePose;

  // initMap() 保存传入的规划器私有句柄，用于注册输入、定时器和显示输出。
  ros::NodeHandle node_;
  // 发布地图相关显示 TF；当前主要广播世界系到 sliding_map 的机身随动平移。
  shared_ptr<tf::TransformBroadcaster> tf_broadcaster_;
  // depth 分支的深度输入，实际话题由 run.launch 的 depth_topic 重映射。
  shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> depth_sub_;
  // depth 分支的传感器位姿输入，与深度图同步后进入 depthPoseCallback()。
  shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> depth_pose_sub_;
  // depth 分支的近似时间同步对象，配置队列长度 100；只在 depth 模式建立。
  SynchronizerImagePose sync_image_pose_;

  // lidar_pose_sub_/cloud_sub_：雷达位姿与点云输入；sliding_map_frame_sub_：机身位姿，用于显示随动坐标。
  ros::Subscriber lidar_pose_sub_, sliding_map_frame_sub_, cloud_sub_;
  // 分别发布 /grid_map/occupancy、occupancy_inflate 和 sliding_map_bbox，供 RViz 核对地图及窗口。
  ros::Publisher map_pub_, map_inf_pub_, sliding_map_bbox_pub_;
  // 发布 /grid_map/unknown 的未知体素显示点云。
  ros::Publisher unknown_pub_;
  // 分别发布反投影观测点调试云、深度输入完成安装外参变换后的位姿；不直接作为执行命令。
  ros::Publisher depth_cloud_pub_, extrinsic_pose_pub_;
  // 均以 0.05 s 为目标周期；前者仅在有待融合观测时更新地图，后者发布显示信息。
  ros::Timer occ_timer_, vis_timer_;

  //
  // 保留的均匀深度噪声接口；当前投影添加噪声语句已注释。
  uniform_real_distribution<double> rand_noise_;
  // 保留的高斯深度噪声接口；当前未在投影中启用。
  normal_distribution<double> rand_noise2_;
  // 保留噪声随机数引擎；不等同于 mockamap 场景生成种子。
  default_random_engine eng_;
};

/* ============================== definition of inline function
 * ============================== */

/// @brief 把全局体素索引逐轴取模映射到固定数组地址；调用方通常应先执行窗口范围检查。
inline int GridMap::toAddress(const Eigen::Vector3i& id) {
  return getLocalIndex(id(0), 0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) +
         getLocalIndex(id(1), 1) * mp_.map_voxel_num_(2) + getLocalIndex(id(2), 2);
}

inline int GridMap::toAddress(int& x, int& y, int& z) {
  return getLocalIndex(x, 0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) +
         getLocalIndex(y, 1) * mp_.map_voxel_num_(2) + getLocalIndex(z, 2);
}

/// @brief 处理负索引的取模结果，确保逻辑世界索引映射到非负缓冲区下标。
inline int GridMap::getLocalIndex(int id, int dim) const {
  int local_id = id % mp_.map_voxel_num_(dim);
  if (local_id < 0) local_id += mp_.map_voxel_num_(dim);
  return local_id;
}

inline int GridMap::toAddressLocal(const Eigen::Vector3i& id_l) const {
  return id_l(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) + id_l(1) * mp_.map_voxel_num_(2) + id_l(2);
}

inline int GridMap::toAddressLocal(int x, int y, int z) const {
  return x * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) + y * mp_.map_voxel_num_(2) + z;
}

inline void GridMap::boundIndex(Eigen::Vector3i& id) {
  Eigen::Vector3i id1;
  id1(0) = max(min(id(0), mp_.map_bound_max_idx_(0)), mp_.map_bound_min_idx_(0));
  id1(1) = max(min(id(1), mp_.map_bound_max_idx_(1)), mp_.map_bound_min_idx_(1));
  id1(2) = max(min(id(2), mp_.map_bound_max_idx_(2)), mp_.map_bound_min_idx_(2));
  id = id1;
}

inline bool GridMap::isUnknown(const Eigen::Vector3i& id) {
  Eigen::Vector3i id1 = id;
  boundIndex(id1);
  return md_.occupancy_buffer_[toAddress(id1)] < mp_.clamp_min_log_ - 1e-3;
}

inline bool GridMap::isUnknown(const Eigen::Vector3d& pos) {
  Eigen::Vector3i idc;
  posToIndex(pos, idc);
  return isUnknown(idc);
}

inline bool GridMap::isKnownFree(const Eigen::Vector3i& id) {
  Eigen::Vector3i id1 = id;
  boundIndex(id1);
  int adr = toAddress(id1);

  // return md_.occupancy_buffer_[adr] >= mp_.clamp_min_log_ &&
  //     md_.occupancy_buffer_[adr] < mp_.min_occupancy_log_;
  return md_.occupancy_buffer_[adr] >= mp_.clamp_min_log_ && md_.occupancy_buffer_inflate_[adr] == 0;
}

inline bool GridMap::isKnownOccupied(const Eigen::Vector3i& id) {
  Eigen::Vector3i id1 = id;
  boundIndex(id1);
  int adr = toAddress(id1);

  return md_.occupancy_buffer_inflate_[adr] == 1;
}

inline void GridMap::setOccupied(Eigen::Vector3d pos) {
  if (!isInMap(pos)) return;

  Eigen::Vector3i id;
  posToIndex(pos, id);

  applyOccupancyUpdate(id, mp_.clamp_max_log_);
}

inline void GridMap::setOccupancy(Eigen::Vector3d pos, double occ) {
  if (occ != 1 && occ != 0) {
    cout << "occ value error!" << endl;
    return;
  }

  if (!isInMap(pos)) return;

  Eigen::Vector3i id;
  posToIndex(pos, id);

  applyOccupancyUpdate(id, occ > 0.5 ? mp_.clamp_max_log_ : mp_.clamp_min_log_);
}

/// @brief 读取原始占用判定：范围外返回 -1；不能把 -1 与已观测空闲的 0 混为一谈。
inline int GridMap::getOccupancy(Eigen::Vector3d pos) {
  if (!isInMap(pos)) return -1;

  Eigen::Vector3i id;
  posToIndex(pos, id);

  return md_.occupancy_buffer_[toAddress(id)] > mp_.min_occupancy_log_ ? 1 : 0;
}

/// @brief 按 yaw 放置前后两个查询中心，任一中心占用或越界就返回相应结果；半径已编码进障碍膨胀。
inline int GridMap::getInflateOccupancy(Eigen::Vector3d pos, double yaw) {
  // 论文式 (3)：机身系两中心隐含为 (+offset,0,0)、(-offset,0,0)，这里不单独创建 s_b 向量。
  // 碰撞尺寸先转入障碍膨胀层，单个圆柱在查询时只剩中心点；两个圆柱的并集对应前后两点查询。
  // yaw 旋转的是两中心的排列方向：front/rear = pos ± offset*(cos(yaw),sin(yaw),0)。
  Eigen::Vector3d heading(std::cos(yaw), std::sin(yaw), 0.0);
  // 论文式 (4)：Rz(yaw)*(±offset,0,0)+pos，化简为 pos±offset*heading。
  Eigen::Vector3d front = pos + mp_.double_cylinder_offset_ * heading;
  Eigen::Vector3d rear = pos - mp_.double_cylinder_offset_ * heading;

  // 论文式 (5)：地图内 0/1 占用的两次查询用短路返回实现 OR/max；本接口额外用 -1 表示越界，不属于论文二值 max 的输入域。
  int front_occ = getInflateOccupancyFromBuffer(front, md_.occupancy_buffer_inflate_);
  // 前中心非零就提前返回（含 1 占用和 -1 越界）；前中心为 0 时才继续检查后中心。
  if (front_occ != 0) return front_occ;

  return getInflateOccupancyFromBuffer(rear, md_.occupancy_buffer_inflate_);
}

/// @brief 对一个世界位置查询膨胀缓冲；范围外返回 -1，调用方需明确如何解释非零结果。
inline int GridMap::getInflateOccupancyFromBuffer(Eigen::Vector3d pos, const std::vector<char>& buffer) {
  if (!isInMap(pos)) return -1;

  Eigen::Vector3i id;
  posToIndex(pos, id);

  return int(buffer[toAddress(id)]);
}

inline int GridMap::getOccupancy(Eigen::Vector3i id) {
  if (!isInMap(id))
    return -1;

  return md_.occupancy_buffer_[toAddress(id)] > mp_.min_occupancy_log_ ? 1 : 0;
}

inline bool GridMap::isInMap(const Eigen::Vector3d& pos) {
  if (pos(0) < mp_.map_min_boundary_(0) + 1e-4 || pos(1) < mp_.map_min_boundary_(1) + 1e-4 ||
      pos(2) < mp_.map_min_boundary_(2) + 1e-4) {
    // cout << "less than min range!" << endl;
    return false;
  }
  if (pos(0) > mp_.map_max_boundary_(0) - 1e-4 || pos(1) > mp_.map_max_boundary_(1) - 1e-4 ||
      pos(2) > mp_.map_max_boundary_(2) - 1e-4) {
    return false;
  }
  return true;
}

inline bool GridMap::isInMap(const Eigen::Vector3i& idx) {
  if (idx(0) < mp_.map_bound_min_idx_(0) || idx(1) < mp_.map_bound_min_idx_(1) ||
      idx(2) < mp_.map_bound_min_idx_(2)) {
    return false;
  }
  if (idx(0) > mp_.map_bound_max_idx_(0) || idx(1) > mp_.map_bound_max_idx_(1) ||
      idx(2) > mp_.map_bound_max_idx_(2)) {
    return false;
  }
  return true;
}

/// @brief 世界米制位置转换为全局整数体素索引；floor 保持负坐标所在体素定义一致。
inline void GridMap::posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id) {
  for (int i = 0; i < 3; ++i) id(i) = floor(pos(i) * mp_.resolution_inv_);
}

inline void GridMap::indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos) {
  for (int i = 0; i < 3; ++i) pos(i) = (id(i) + 0.5) * mp_.resolution_;
}

inline void GridMap::inflatePoint(const Eigen::Vector3i& pt, int inf_step_xy, int inf_step_z_up, int inf_step_z_down, vector<Eigen::Vector3i>& pts) {

  /* ---------- + shape inflate ---------- */
  // for (int x = -step; x <= step; ++x)
  // {
  //   if (x == 0)
  //     continue;
  //   pts[num++] = Eigen::Vector3i(pt(0) + x, pt(1), pt(2));
  // }
  // for (int y = -step; y <= step; ++y)
  // {
  //   if (y == 0)
  //     continue;
  //   pts[num++] = Eigen::Vector3i(pt(0), pt(1) + y, pt(2));
  // }
  // for (int z = -1; z <= 1; ++z)
  // {
  //   pts[num++] = Eigen::Vector3i(pt(0), pt(1), pt(2) + z);
  // }

  /* ---------- all inflate ---------- */
  pts.clear();
  for (int x = -inf_step_xy; x <= inf_step_xy; ++x)
    for (int y = -inf_step_xy; y <= inf_step_xy; ++y)
    {
      if (std::sqrt(x * x + y * y) > inf_step_xy)
        continue;

      for (int z = -inf_step_z_down; z <= inf_step_z_up; ++z) {
        pts.push_back(Eigen::Vector3i(pt(0) + x, pt(1) + y, pt(2) + z));
      }
    }
}

inline double GridMap::getResolution() { return mp_.resolution_; }

#endif
