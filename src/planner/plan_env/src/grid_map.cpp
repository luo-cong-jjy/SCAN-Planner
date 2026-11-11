/// @file
/// 机器人附近的三维滑动占用地图，由 Manager 在规划节点进程内创建。
/// lidar 使用点云与最新传感器位姿，depth 使用近似同步的深度图与位姿；最终都通过射线更新 log-odds。
/// 固定缓冲区通过全局体素索引取模复用，维护占用值、膨胀层与膨胀计数；位置/尺寸单位 m。
/// 搜索、优化和安全回调查询此地图，显示话题只暴露其可视化结果。
/// 阅读重点：updateSlidingMap 的地址复用、raycastProcess 的证据融合、applyOccupancyUpdate 的膨胀贡献维护。
/// 机制、阅读顺序与面试追问见 docs/面试/算法原理与面试问题.md#滑动局部地图的原理与面试重点。
#include "plan_env/grid_map.h"
#include <cmath>
#include <limits>
#include <string>
#include <tf/transform_broadcaster.h>

/// @brief 读取私有 grid_map 参数，建立窗口/缓冲区、输入订阅、地图更新和显示定时器。
/// 外参矩阵在此定义；真机位姿是否需要再乘外参由 need_extrinsic 控制。
void GridMap::initMap(ros::NodeHandle &nh)
{
  node_ = nh;

  /* get parameter */
  double x_size, y_size, z_size;
  // 本段读取启动配置：下方“正常默认”来自当前 launch/XML，实际运行以参数服务器为准。
  // param() 的末项仅是缺参回退；getParam()/必需参数封装没有这一回退，具体见各行说明。
  // 参数读取：单个体素的边长，单位 m，必须为正；减小它会增加地图内存、更新和搜索开销。
  // 配置来源：advanced_param.xml → /scan_planner_node/grid_map/resolution；正常默认启动取 0.05。
  // 缺参回退：-1.0；只有参数服务器中没有该键时才使用。
  node_.param("grid_map/resolution", mp_.resolution_, -1.0);
  // 参数读取：滑动窗口在世界 x 方向的长度，单位 m。
  // 配置来源：advanced_param.xml → /scan_planner_node/grid_map/sliding_map_size_x；正常默认启动取
  // 10.0。
  // 缺参回退：-1.0；只有参数服务器中没有该键时才使用。
  node_.param("grid_map/sliding_map_size_x", x_size, -1.0);
  // 参数读取：滑动窗口在世界 y 方向的长度，单位 m。
  // 配置来源：advanced_param.xml → /scan_planner_node/grid_map/sliding_map_size_y；正常默认启动取
  // 10.0。
  // 缺参回退：-1.0；只有参数服务器中没有该键时才使用。
  node_.param("grid_map/sliding_map_size_y", y_size, -1.0);
  // 参数读取：滑动窗口在世界 z 方向的长度，单位 m；不是可视化裁剪高度。
  // 配置来源：advanced_param.xml → /scan_planner_node/grid_map/sliding_map_size_z；正常默认启动取
  // 5.0。
  // 缺参回退：-1.0；只有参数服务器中没有该键时才使用。
  node_.param("grid_map/sliding_map_size_z", z_size, -1.0);
  // 参数读取：单轮融合的 x 方向半范围（m）；未配置时取窗口 x 尺寸的一半。
  // 配置位置：私有 /scan_planner_node/grid_map/local_update_range_x；当前主 launch 未覆盖此键，取下
  // 面的缺参回退。
  // 缺参回退：x_size/2.0；只有参数服务器中没有该键时才使用。
  node_.param("grid_map/local_update_range_x", mp_.local_update_range_(0), x_size/2.0);
  // 参数读取：单轮融合的 y 方向半范围（m）；未配置时取窗口 y 尺寸的一半。
  // 配置位置：私有 /scan_planner_node/grid_map/local_update_range_y；当前主 launch 未覆盖此键，取下
  // 面的缺参回退。
  // 缺参回退：y_size/2.0；只有参数服务器中没有该键时才使用。
  node_.param("grid_map/local_update_range_y", mp_.local_update_range_(1), y_size/2.0);
  // 参数读取：单轮融合的 z 方向半范围（m）；未配置时取窗口 z 尺寸的一半。
  // 配置位置：私有 /scan_planner_node/grid_map/local_update_range_z；当前主 launch 未覆盖此键，取下
  // 面的缺参回退。
  // 缺参回退：z_size/2.0；只有参数服务器中没有该键时才使用。
  node_.param("grid_map/local_update_range_z", mp_.local_update_range_(2), z_size/2.0);
  
  // 参数读取：相对障碍体素向上膨胀的距离，单位 m；转换为体素数后形成垂直碰撞裕量。
  // 配置来源：advanced_param.xml → /scan_planner_node/grid_map/obstacles_inflation_z_up；正常默认启
  // 动取 0.1。
  // 缺参回退：-1.0；只有参数服务器中没有该键时才使用。
  node_.param("grid_map/obstacles_inflation_z_up", mp_.obstacles_inflation_z_up, -1.0);
  // 参数读取：相对障碍体素向下膨胀的距离，单位 m；可与向上裕量独立配置。
  // 配置来源：advanced_param.xml → /scan_planner_node/grid_map/obstacles_inflation_z_down；正常默认
  // 启动取 0.1。
  // 缺参回退：-1.0；只有参数服务器中没有该键时才使用。
  node_.param("grid_map/obstacles_inflation_z_down", mp_.obstacles_inflation_z_down, -1.0);
  // 参数读取：双圆柱碰撞模型的水平半径，单位 m；rebuildInflationOffsets() 用它构造障碍物的水平膨胀
  // 偏移。
  // 配置来源：advanced_param.xml → /scan_planner_node/grid_map/double_cylinder_radius；正常默认启动
  // 取 0.25。
  // 缺参回退：-1.0；只有参数服务器中没有该键时才使用。
  node_.param("grid_map/double_cylinder_radius", mp_.double_cylinder_radius_, -1.0);
  // 参数读取：前后两个圆柱中心距机器人参考点的偏移量，单位 m；两中心间距为该值的 2 倍。
  // getInflateOccupancy(pos,yaw) 在 pos ± offset*(cos(yaw),sin(yaw),0) 查询膨胀占用。
  // 配置来源：advanced_param.xml → /scan_planner_node/grid_map/double_cylinder_offset；正常默认启动
  // 取 0.18。
  // 缺参回退：0.0；只有参数服务器中没有该键时才使用。
  node_.param("grid_map/double_cylinder_offset", mp_.double_cylinder_offset_, 0.0);
  // 参数读取：是否启用局部窗口平移；false 时窗口不跟随传感器移动。
  // 配置位置：私有 /scan_planner_node/grid_map/map_sliding_en；当前主 launch 未覆盖此键，取下面的缺
  // 参回退。
  // 缺参回退：true；只有参数服务器中没有该键时才使用。
  node_.param("grid_map/map_sliding_en", mp_.map_sliding_en_, true);
  // 参数读取：触发窗口平移的位移阈值，单位 m；初始化时向上取整为体素数，且至少为 1 个体素。
  // updateSlidingMap() 按中心体素索引差的最大绝对分量判断，不是平移更新周期。
  // 配置来源：advanced_param.xml → /scan_planner_node/grid_map/map_sliding_thresh；正常默认启动取
  // 0.2。
  // 缺参回退：mp_.resolution_；只有参数服务器中没有该键时才使用。
  node_.param("grid_map/map_sliding_thresh", mp_.map_sliding_thresh_, mp_.resolution_);

  // 参数读取：水平焦距 fx，单位 px，必须为正。
  // 配置来源：advanced_param.xml → /scan_planner_node/grid_map/fx；正常默认启动取 387.229248046875
  // 。
  // 缺参回退：-1.0；只有参数服务器中没有该键时才使用。
  node_.param("grid_map/fx", mp_.fx_, -1.0);
  // 参数读取：垂直焦距 fy，单位 px，必须为正。
  // 配置来源：advanced_param.xml → /scan_planner_node/grid_map/fy；正常默认启动取 387.229248046875
  // 。
  // 缺参回退：-1.0；只有参数服务器中没有该键时才使用。
  node_.param("grid_map/fy", mp_.fy_, -1.0);
  // 参数读取：主点横坐标 cx，单位 px。
  // 配置来源：advanced_param.xml → /scan_planner_node/grid_map/cx；正常默认启动取 321.04638671875。
  // 缺参回退：-1.0；只有参数服务器中没有该键时才使用。
  node_.param("grid_map/cx", mp_.cx_, -1.0);
  // 参数读取：主点纵坐标 cy，单位 px。
  // 配置来源：advanced_param.xml → /scan_planner_node/grid_map/cy；正常默认启动取
  // 243.44969177246094。
  // 缺参回退：-1.0；只有参数服务器中没有该键时才使用。
  node_.param("grid_map/cy", mp_.cy_, -1.0);

  // 参数读取：有效障碍深度的上限，单位 m；超出上限或原始深度为 0 时，代码生成超量程射线以更新自由空
  // 间， 而非将该像素直接作为有效障碍端点；最终射线长度仍受 max_ray_length 限制。
  // 配置来源：advanced_param.xml → /scan_planner_node/grid_map/depth_filter_maxdist；正常默认启动取
  // 3.0。
  // 缺参回退：-1.0；只有参数服务器中没有该键时才使用。
  node_.param("grid_map/depth_filter_maxdist", mp_.depth_filter_maxdist_, -1.0);
  // 参数读取：有效深度下限，单位 m；非零且小于该值的像素直接跳过。
  // 配置来源：advanced_param.xml → /scan_planner_node/grid_map/depth_filter_mindist；正常默认启动取
  // 0.3。
  // 缺参回退：-1.0；只有参数服务器中没有该键时才使用。
  node_.param("grid_map/depth_filter_mindist", mp_.depth_filter_mindist_, -1.0);
  // 参数读取：深度图四周跳过的边缘宽度，单位 px；采样范围由该边界内缩。
  // 配置来源：advanced_param.xml → /scan_planner_node/grid_map/depth_filter_margin；正常默认启动取
  // 1。
  // 缺参回退：-1；只有参数服务器中没有该键时才使用。
  node_.param("grid_map/depth_filter_margin", mp_.depth_filter_margin_, -1);
  // 参数读取：整数深度到米的比例因子，必须为正：Z[m]=raw_depth/k_depth_scaling_factor，1000 对应毫
  // 米数据。输入为 32FC1 时，depthPoseCallback() 先按该因子转换为 16 位整数，再进行上述反投影。
  // 配置来源：advanced_param.xml → /scan_planner_node/grid_map/k_depth_scaling_factor；正常默认启动
  // 取 1000.0。
  // 缺参回退：-1.0；只有参数服务器中没有该键时才使用。
  node_.param("grid_map/k_depth_scaling_factor", mp_.k_depth_scaling_factor_, -1.0);
  // 参数读取：横向和纵向的像素采样步长，须为正整数；2 表示每个方向隔一个像素采样，减少反投影开销。
  // 配置来源：advanced_param.xml → /scan_planner_node/grid_map/skip_pixel；正常默认启动取 2。
  // 缺参回退：-1；只有参数服务器中没有该键时才使用。
  node_.param("grid_map/skip_pixel", mp_.skip_pixel_, -1);

  // 参数读取：命中证据的更新概率，logit(p_hit)>0，使相应体素更倾向占用；不是障碍物检测准确率。
  // 配置来源：advanced_param.xml → /scan_planner_node/grid_map/p_hit；正常默认启动取 0.85。
  // 缺参回退：-1.0；只有参数服务器中没有该键时才使用。
  node_.param("grid_map/p_hit", mp_.p_hit_, -1.0);
  // 参数读取：未命中/自由空间证据的更新概率，logit(p_miss)<0，使体素更倾向空闲；不是 1-p_hit。
  // 配置来源：advanced_param.xml → /scan_planner_node/grid_map/p_miss；正常默认启动取 0.3。
  // 缺参回退：-1.0；只有参数服务器中没有该键时才使用。
  node_.param("grid_map/p_miss", mp_.p_miss_, -1.0);
  // 参数读取：占用置信度的截断下限，防止空闲证据无限累计；不是观测有效性的概率阈值。
  // 配置来源：advanced_param.xml → /scan_planner_node/grid_map/p_min；正常默认启动取 0.12。
  // 缺参回退：-1.0；只有参数服务器中没有该键时才使用。
  node_.param("grid_map/p_min", mp_.p_min_, -1.0);
  // 参数读取：占用置信度的截断上限，防止命中证据无限累计，并允许后续未命中观测逐步清除占用。
  // 配置来源：advanced_param.xml → /scan_planner_node/grid_map/p_max；正常默认启动取 0.98。
  // 缺参回退：-1.0；只有参数服务器中没有该键时才使用。
  node_.param("grid_map/p_max", mp_.p_max_, -1.0);
  // 参数读取：体素被判为占用的概率阈值；代码比较其 log-odds，并据此维护膨胀占用层。
  // 配置来源：advanced_param.xml → /scan_planner_node/grid_map/p_occ；正常默认启动取 0.8。
  // 缺参回退：-1.0；只有参数服务器中没有该键时才使用。
  node_.param("grid_map/p_occ", mp_.p_occ_, -1.0);
  // 参数读取：地图融合的最大射线长度，单位 m；更远观测截断到该范围，截断点不按真实障碍命中处理。
  // 配置来源：advanced_param.xml → /scan_planner_node/grid_map/max_ray_length；正常默认启动取 5.0。
  // 缺参回退：-0.1；只有参数服务器中没有该键时才使用。
  node_.param("grid_map/max_ray_length", mp_.max_ray_length_, -0.1);

  // 参数读取：显示点云的高度上限相对传感器位置的偏移，单位 m：高于 ray_pos.z+vis_height 的点不发布
  // 显示。用于占用/膨胀/未知点云显示裁剪，不改变内部占用地图或规划碰撞查询的高度范围。
  // 配置来源：advanced_param.xml → /scan_planner_node/grid_map/vis_height；正常默认启动取 0.3。
  // 缺参回退：0.3；只有参数服务器中没有该键时才使用。
  node_.param("grid_map/vis_height", mp_.vis_height_, 0.3);
  // 参数读取：地图耗时显示开关；当前对应统计打印代码已注释，设置 true 不会自动恢复。
  // 配置位置：私有 /scan_planner_node/grid_map/show_occ_time；当前主 launch 未覆盖此键，取下面的缺
  // 参回退。
  // 缺参回退：false；只有参数服务器中没有该键时才使用。
  node_.param("grid_map/show_occ_time", mp_.show_occ_time_, false);

  // 参数读取：地图与几何显示使用的世界参考系名称。
  // 配置位置：私有 /scan_planner_node/grid_map/frame_id；当前主 launch 未覆盖此键，取下面的缺参回退
  // 。
  // 缺参回退：string("world")；只有参数服务器中没有该键时才使用。
  node_.param("grid_map/frame_id", mp_.frame_id_, string("world"));
  // 参数读取：供 RViz 跟随的平移坐标系名称，默认 sliding_map。
  // 配置位置：私有 /scan_planner_node/grid_map/sliding_map_frame_id；当前主 launch 未覆盖此键，取下
  // 面的缺参回退。
  // 缺参回退：string("sliding_map")；只有参数服务器中没有该键时才使用。
  node_.param("grid_map/sliding_map_frame_id", mp_.sliding_map_frame_id_, string("sliding_map"));
  // 参数读取：初始地图 z 原点和更新范围处理使用的高度（m），不是机身高度测量。
  // 配置位置：私有 /scan_planner_node/grid_map/ground_height；当前主 launch 未覆盖此键，取下面的缺
  // 参回退。
  // 缺参回退：0.0；只有参数服务器中没有该键时才使用。
  node_.param("grid_map/ground_height", mp_.ground_height_, 0.0);

  // 参数读取：显式字符串类型；GridMap::initMap() 据此选择 lidar 或 depth 的订阅/融合分支。
  // 配置来源：advanced_param.xml → /scan_planner_node/grid_map/sensor_type；正常默认启动取 lidar。
  // 缺参回退：string("lidar")；只有参数服务器中没有该键时才使用。
  node_.param("grid_map/sensor_type", mp_.sensor_type_, string("lidar"));
  // 参数读取：GridMap::cloudCallback() 的坐标系开关；不改变 PointCloud2 内容，只决定如何解释和变换
  // XYZ。
  // 配置来源：advanced_param.xml → /scan_planner_node/grid_map/cloud_is_world；正常默认启动取 true
  // 。
  // 缺参回退：true；只有参数服务器中没有该键时才使用。
  node_.param("grid_map/cloud_is_world", mp_.cloud_is_world_, true);
  // 参数读取：是否将输入位姿再乘传感器外参；当前随 is_real_world 取值。外参矩阵在
  // GridMap::initMap() 中定义，分别由 sensorPoseCallback()/depthPoseCallback() 使用。
  // 配置来源：advanced_param.xml → /scan_planner_node/grid_map/need_extrinsic；正常默认启动取 false
  // 。
  // 缺参回退：true；只有参数服务器中没有该键时才使用。
  node_.param("grid_map/need_extrinsic", mp_.need_extrinsic_, true);

  mp_.lidar_extrinsic_ <<
      1.0, 0.0, 0.0, -0.01100,
      0.0, 1.0, 0.0, -0.02329,
      0.0, 0.0, 1.0,  0.04412,
      0.0, 0.0, 0.0,  1.00000;

  mp_.depth_extrinsic_ <<
      0.0,  0.707107, 0.707107, -0.15170,
     -1.0,  0.000000, 0.000000,  0.00000,
      0.0, -0.707107, 0.707107,  0.07510,
      0.0,  0.000000, 0.000000,  1.00000;
      
  if (mp_.sensor_type_ != "lidar" && mp_.sensor_type_ != "depth")
  {
    ROS_ERROR_STREAM("[GridMap] invalid grid_map/sensor_type: " << mp_.sensor_type_
                                                                << ", fallback to lidar.");
    mp_.sensor_type_ = "lidar";
  }

  // 保存体素边长的倒数（1/m），用于将米制坐标换算成体素索引；例如边长 0.05 m 时，倒数为 20。
  mp_.resolution_inv_ = 1 / mp_.resolution_;
  // 设置初始地图窗口的最小角坐标（m）：XY 关于世界原点居中，Z 底面从 ground_height_ 开始。
  // 例如窗口长宽均为 10 m，初始最小 X/Y 为 -5 m；窗口滑动后，此原点会随地图边界更新。
  mp_.map_origin_ = Eigen::Vector3d(-x_size / 2.0, -y_size / 2.0, mp_.ground_height_);
  // 保存窗口沿 XYZ 的尺寸（m），不是最大角坐标；后续用尺寸/分辨率计算各轴体素数。
  mp_.map_size_ = Eigen::Vector3d(x_size, y_size, z_size);

  // 把概率配置转为可累加的 log-odds；p_hit/p_miss 分别提供正/负证据，阈值与截断值单独保存。
  // logit 在 grid_map.h 中定义为宏：L=ln(p/(1-p))；后续融合用 L 累加证据，而不是直接累加概率。
  // 以下只准备常量，并未在这里更新任何观测体素；默认数值对应当前 advanced_param.xml。
  // 命中更新量：p_hit=0.85 → L≈+1.735，后续增加该体素的占用倾向。
  mp_.prob_hit_log_ = logit(mp_.p_hit_);
  // 未命中更新量：p_miss=0.30 → L≈-0.847，后续降低该体素的占用倾向。
  mp_.prob_miss_log_ = logit(mp_.p_miss_);
  // 融合结果的截断下限：p_min=0.12 → L≈-1.992，限制空闲证据的累计幅度。
  mp_.clamp_min_log_ = logit(mp_.p_min_);
  // 融合结果的截断上限：p_max=0.98 → L≈+3.892，限制占用证据，便于之后的空闲观测纠正。
  mp_.clamp_max_log_ = logit(mp_.p_max_);
  // 占用判定阈值：p_occ=0.80 → L≈+1.386；实际判定使用体素 L 大于此值，不是每次命中都直接判占用。
  mp_.min_occupancy_log_ = logit(mp_.p_occ_);
  // 未知状态的数值偏移量，不是 1% 概率；下面将新体素初始化为 clamp_min_log_-0.01，与已观测空闲区分。
  mp_.unknown_flag_ = 0.01;
  // 将窗口平移门槛由米换成格数：位移(m)×分辨率倒数(1/m)，ceil 向上取整，再至少保留 1 格。
  // static_cast<int>(表达式) 是 C++ 显式类型转换语法，不是普通函数；把 ceil 返回的浮点结果变成整数。
  // 例如 0.21/0.05=4.2 → ceil 得 5.0 → 转成 int 为 5；向上取整由 ceil 完成，不是 static_cast 完成。
  // 当前默认 0.2/0.05=4 格，后续按中心索引位移判断是否移动窗口，不是每 4 次回调移动一次。
  mp_.map_sliding_thresh_vox_ = std::max(1, static_cast<int>(std::ceil(mp_.map_sliding_thresh_ * mp_.resolution_inv_)));

  // 打印转换后的更新量、截断值和占用阈值用于启动核对；只输出日志，不触发地图融合。
  cout << "hit: " << mp_.prob_hit_log_ << endl;
  cout << "miss: " << mp_.prob_miss_log_ << endl;
  cout << "min log: " << mp_.clamp_min_log_ << endl;
  cout << "max: " << mp_.clamp_max_log_ << endl;
  cout << "thresh log: " << mp_.min_occupancy_log_ << endl;

  // 沿 XYZ 分别计算需要多少体素；向上取整覆盖配置尺寸，默认 (10,10,5)/0.05 得 (200,200,100)。
  // i=0/1/2 分别对应 X/Y/Z；map_voxel_num_ 存的是各轴格数，不是米制长度。
  for (int i = 0; i < 3; ++i)
    mp_.map_voxel_num_(i) = ceil(mp_.map_size_(i) / mp_.resolution_);

  // 先按名义尺寸给出窗口世界坐标范围：最小角=origin，最大角=origin+size（单位 m）。
  mp_.map_min_boundary_ = mp_.map_origin_;
  mp_.map_max_boundary_ = mp_.map_origin_ + mp_.map_size_;
  // 将最小角世界坐标转为全局体素索引，内部逐轴 floor(坐标/分辨率)；索引可能为负，不是数组地址。
  posToIndex(mp_.map_origin_, mp_.map_bound_min_idx_);
  // 最后一格索引=首格索引+格数-1；Eigen::Vector3i::Ones() 为 (1,1,1)，逐轴减 1。
  // 例如 X 轴从 -100 开始共 200 格，最后一格为 99，不是 100。
  mp_.map_bound_max_idx_ = mp_.map_bound_min_idx_ + mp_.map_voxel_num_ - Eigen::Vector3i::Ones();
  // 建立滑动窗口的中心索引锚点：最小索引+各轴格数的一半，整数分量除法；虽名为 origin_idx_，它不是最小角索引。
  mp_.map_origin_idx_ = mp_.map_bound_min_idx_ + mp_.map_voxel_num_ / 2;
  // 用上述中心和格数重算索引边界，再转换回世界边界并刷新 map_origin_，使窗口边缘对齐体素划分。
  // 因格数已向上取整、起点索引已向下取整，对齐后的范围可能与名义米制范围略有差别。
  updateMapBoundaryFromIndex();

  // initialize data buffers

  // 固定窗口控制存储规模：默认 10×10×5 m、0.05 m 分辨率为 200×200×100=400 万体素，长程移动不追加整张全局体素图。
  // 仅 double 占用数组约 32 MB，下面还有膨胀、计数等缓冲区；滑动复用空间是为了保持局部高分辨率而限制总容量。
  int buffer_size = mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2);

  // vector<T>(数量, 初始值)：创建指定数量的元素并统一赋初值；以下地图数组以相同槽位对应同一个体素。
  // 原始占用 log-odds：设为比空闲截断下限更小的值，标记尚未观测，而不是初始化成“已知空闲”。
  md_.occupancy_buffer_ = vector<double>(buffer_size, mp_.clamp_min_log_ - mp_.unknown_flag_);
  // 膨胀占用标志：0 表示当前未标记膨胀障碍；仅凭这个 0 不能证明该格已观测为空闲。
  md_.occupancy_buffer_inflate_ = vector<char>(buffer_size, 0);
  // 膨胀覆盖计数：记录各格被多少个占用源覆盖；删除一个源时仍保留其他源的贡献，初始均为 0。
  md_.occupancy_buffer_inflate_cnt_ = vector<int>(buffer_size, 0);
  // 根据圆柱半径、上下膨胀距离与分辨率，预计算邻域体素偏移，供后续占用变化时复用；此处尚未加入障碍。
  rebuildInflationOffsets();

  // 本轮每格接收的 hit+miss 总次数，先清零；只把首次入缓存的格子放入待融合队列。
  md_.count_hit_and_miss_ = vector<short>(buffer_size, 0);
  // 上述总次数中的 hit 次数；与 total-hit 得到的 miss 次数比较，选择本轮正/负更新，融合后清零。
  md_.count_hit_ = vector<short>(buffer_size, 0);
  // 射线端点处理标记：按当前 raycast_num_ 标识本轮已处理的端点，减少重复端点的后续射线遍历。
  md_.flag_rayend_ = vector<char>(buffer_size, -1);
  // 射线沿途体素的遍历标记：用于本轮多条射线共用区域的去重/提前结束；-1 为初始未处理标记。
  md_.flag_traverse_ = vector<char>(buffer_size, -1);

  // 射线融合轮次从 0 开始，每次处理非空投影点集时递增；它不是本帧射线条数或 ROS 回调频率。
  md_.raycast_num_ = 0;

  // 按 640×480 深度图、横纵均隔 skip_pixel 采样的规模建立初始点缓冲；默认步长 2，对应 76800 个点位。
  // resize 改变实际元素个数，但这些位置尚不是有效观测；这里只能作为初始尺寸，不能保证适配任意图像分辨率。
  // lidar 分支在需要时可 push_back 扩展；后续只融合下面 proj_points_cnt 指定的有效前缀。
  md_.proj_points_.resize(640 * 480 / mp_.skip_pixel_ / mp_.skip_pixel_);
  // 当前有效投影点数初始化为 0；收到点云或完成深度反投影后再填点计数，不等于上面的缓冲长度。
  md_.proj_points_cnt = 0;

  /* init callback */
  // 创建 TF 广播器并交给智能指针管理；后续 publishSlidingMapFrame() 用它发送地图显示坐标变换。
  // 此处只创建对象，不会让地图滑动；实际体素窗口更新由 updateSlidingMap() 完成。
  // 成员指针在 initMap 返回后仍可供显示回调复用，最后一个强所有者释放时自动销毁广播器。
  // TF 类本身不强制使用 shared_ptr；当前未见对外共享此指针，也可另行设计为独占指针或按值成员。
  tf_broadcaster_.reset(new tf::TransformBroadcaster);

  // 以下 this 均指当前执行 initMap() 的 GridMap 对象，不是 node_ 句柄、订阅器或同步器。
  // 回调需同时指定“调用哪个成员函数”和“在哪个地图对象上调用”，才能更新该对象自己的 md_/mp_。
  if (mp_.sensor_type_ == "depth")
  {
    // 深度分支输入：/grid_map/depth 经 remap 接实际深度图，队列 50，等待与位姿近似同步。
    depth_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(node_, "/grid_map/depth", 50));
    // 深度分支位姿输入：/grid_map/sensor_pose 经 remap 接相机/LIO 位姿，队列 100。
    depth_pose_sub_.reset(
        new message_filters::Subscriber<nav_msgs::Odometry>(node_, "/grid_map/sensor_pose", 100));

    sync_image_pose_.reset(new message_filters::Synchronizer<SyncPolicyImagePose>(
        SyncPolicyImagePose(100), *depth_sub_, *depth_pose_sub_));
    // 配对完成才调用 depthPoseCallback()；同步策略队列为 100，不是地图更新频率。
    // registerCallback 是给双消息同步器登记处理函数，不是再次创建单话题订阅或定时器。
    // boost::bind 固定成员函数与当前 this，_1/_2 占位等待配对后的深度图/位姿；此处不会立即执行回调。
    // 同步器触发时相当于 this->depthPoseCallback(image, pose)，并非在同步器对象上调用 GridMap 的成员函数。
    sync_image_pose_->registerCallback(boost::bind(&GridMap::depthPoseCallback, this, _1, _2));
  }
  else if (mp_.sensor_type_ == "lidar")
  {
    lidar_pose_sub_ =
        // 运行连接（接收入口）：GridMap 的传感器位姿输入，经父 launch 重映射到模拟/真实传感器位姿，
        // 用于观测坐标和射线原点。
        // 队列容量：50 条消息，不是发布/规划频率。 到达后交给 sensorPoseCallback。
        node_.subscribe<nav_msgs::Odometry>("/grid_map/sensor_pose", 50, &GridMap::sensorPoseCallback, this);
    cloud_sub_ =
        // 运行连接（接收入口）：GridMap 的激光观测输入，经 remap 接到当前 cloud_topic；不会直接消费
        // 模拟全局环境真值。
        // 队列容量：10 条消息，不是发布/规划频率。 到达后交给 cloudCallback。
        node_.subscribe<sensor_msgs::PointCloud2>("/grid_map/cloud", 10, &GridMap::cloudCallback, this);
  }

  sliding_map_frame_sub_ =
      // 运行连接（接收入口）：GridMap 的机身位置输入，经 remap 选择机身话题，主要用于 sliding_map
      // 显示坐标随动。
      // 队列容量：50 条消息，不是发布/规划频率。 到达后交给 slidingMapFrameCallback。
      node_.subscribe<nav_msgs::Odometry>("/grid_map/body_pose", 50, &GridMap::slidingMapFrameCallback, this);

  // 定时任务：消费待融合观测，周期 0.05 s；没有新观测标志时不重复更新概率。
  // 与 subscribe/bind 中 this 的用途相同：绑定当前地图实例；this 是借用地址，不增加 shared_ptr 所有权。
  occ_timer_ = node_.createTimer(ros::Duration(0.05), &GridMap::updateOccupancyCallback, this);
  // 定时任务：发布地图/窗口/随动 TF 等显示信息，周期 0.05 s，不改变控制指令。
  vis_timer_ = node_.createTimer(ros::Duration(0.05), &GridMap::visCallback, this);

  // 运行连接（输出入口）：GridMap 的原始占用体素显示点云，供 RViz 检查；实际碰撞查询直接访问内存地
  // 图。
  // 队列容量：10 条消息，不是发布/规划频率。
  map_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/grid_map/occupancy", 10);
  // 运行连接（输出入口）：GridMap 的障碍膨胀显示点云，便于核对机器人几何和占用裕量。
  // 队列容量：10 条消息，不是发布/规划频率。
  map_inf_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/grid_map/occupancy_inflate", 10);
  // 运行连接（输出入口）：GridMap 的当前窗口线框 Marker，供观察窗口是否跟随位置移动。
  // 队列容量：10 条消息，不是发布/规划频率。
  sliding_map_bbox_pub_ = node_.advertise<visualization_msgs::Marker>("/grid_map/sliding_map_bbox", 10);

  // 运行连接（输出入口）：GridMap 的未知体素显示，不是独立感知输入。
  // 队列容量：10 条消息，不是发布/规划频率。
  unknown_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/grid_map/unknown", 10);
  // 运行连接（输出入口）：GridMap 的反投影/观测点调试显示，用于核对几何输入。
  // 队列容量：10 条消息，不是发布/规划频率。
  depth_cloud_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/grid_map/depth_cloud", 10);
  // 运行连接（输出入口）：深度位姿完成安装外参处理后的调试输出，不是新的定位解算。
  // 队列容量：10 条消息，不是发布/规划频率。
  extrinsic_pose_pub_ = node_.advertise<nav_msgs::Odometry>("/grid_map/sensor_pose_extrinsic", 10);

  md_.occ_need_update_ = false;
  md_.use_cloud_update_ = false;
  md_.has_first_depth_ = false;
  md_.has_ray_pose_ = false;
  md_.has_cloud_ = false;
  md_.image_cnt_ = 0;
  md_.ray_pos_.setZero();
  md_.sliding_map_frame_pos_.setZero();
  md_.ray_q_ = Eigen::Quaterniond::Identity();

  md_.fuse_time_ = 0.0;
  md_.update_num_ = 0;
  md_.max_fuse_time_ = 0.0;
  md_.local_bound_min_ = mp_.map_bound_min_idx_;
  md_.local_bound_max_ = mp_.map_bound_max_idx_;

  // rand_noise_ = uniform_real_distribution<double>(-0.2, 0.2);
  // rand_noise2_ = normal_distribution<double>(0, 0.2);
  // random_device rd;
  // eng_ = default_random_engine(rd());
}

/// @brief 由窗口体素索引边界重算世界坐标包围盒，使范围查询与滑动后的索引保持对应。
void GridMap::updateMapBoundaryFromIndex()
{
  mp_.map_bound_min_idx_ = mp_.map_origin_idx_ - mp_.map_voxel_num_ / 2;
  mp_.map_bound_max_idx_ = mp_.map_bound_min_idx_ + mp_.map_voxel_num_ - Eigen::Vector3i::Ones();

  mp_.map_min_boundary_ = mp_.map_bound_min_idx_.cast<double>() * mp_.resolution_;
  mp_.map_max_boundary_ = (mp_.map_bound_max_idx_.cast<double>() + Eigen::Vector3d::Ones()) * mp_.resolution_;
  mp_.map_origin_ = mp_.map_min_boundary_;
}

/// @brief 把水平圆柱半径及上下膨胀距离离散为相对体素偏移，供占用变化时增量更新膨胀层。
void GridMap::rebuildInflationOffsets()
{
  // 在每个原始占用体素周围标记圆柱形偏移：XY 按半径扩展，Z 按配置的上下距离扩展。
  // 圆柱是单个占用源的膨胀模板；多个源的模板取并集，因此整面墙膨胀后仍是加厚的墙，不是单根圆柱。
  // 增量占用更新用这些偏移维护膨胀标志；姿态查询只读中心对应格，不再遍历完整圆柱体积。
  const double double_radius = std::max(0.0, mp_.double_cylinder_radius_);
  const int inf_step_xy = ceil(double_radius / mp_.resolution_);
  const int inf_step_z_up = ceil(mp_.obstacles_inflation_z_up / mp_.resolution_);
  const int inf_step_z_down = ceil(mp_.obstacles_inflation_z_down / mp_.resolution_);

  md_.inflate_offsets_.clear();
  for (int x = -inf_step_xy; x <= inf_step_xy; ++x)
    for (int y = -inf_step_xy; y <= inf_step_xy; ++y)
    {
      Eigen::Vector2d offset_xy(x * mp_.resolution_, y * mp_.resolution_);
      if (offset_xy.norm() >= double_radius)
        continue;

      for (int z = -inf_step_z_down; z <= inf_step_z_up; ++z)
        md_.inflate_offsets_.push_back(Eigen::Vector3i(x, y, z));
    }
}

/// @brief 清空整张局部地图及相关缓存；窗口跨越大于自身尺寸时使用，旧观测不能继续复用。
void GridMap::resetAllMapData()
{
  std::fill(md_.occupancy_buffer_.begin(), md_.occupancy_buffer_.end(), mp_.clamp_min_log_ - mp_.unknown_flag_);
  std::fill(md_.occupancy_buffer_inflate_.begin(), md_.occupancy_buffer_inflate_.end(), 0);
  std::fill(md_.occupancy_buffer_inflate_cnt_.begin(), md_.occupancy_buffer_inflate_cnt_.end(), 0);
  std::fill(md_.count_hit_and_miss_.begin(), md_.count_hit_and_miss_.end(), 0);
  std::fill(md_.count_hit_.begin(), md_.count_hit_.end(), 0);
  std::fill(md_.flag_rayend_.begin(), md_.flag_rayend_.end(), -1);
  std::fill(md_.flag_traverse_.begin(), md_.flag_traverse_.end(), -1);
  std::queue<Eigen::Vector3i> empty;
  std::swap(md_.cache_voxel_, empty);
}

/// @brief 由复用缓冲区地址和当前窗口边界恢复全局体素索引；取模地址本身不唯一标识世界位置。
void GridMap::hashIdToGlobalIndex(int addr, Eigen::Vector3i& id_g) const
{
  Eigen::Vector3i id_l;
  id_l(0) = addr / (mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2));
  id_l(1) = (addr - id_l(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2)) / mp_.map_voxel_num_(2);
  id_l(2) = addr - id_l(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) -
            id_l(1) * mp_.map_voxel_num_(2);

  for (int i = 0; i < 3; ++i)
  {
    const int min_l = getLocalIndex(mp_.map_bound_min_idx_(i), i);
    int dist = id_l(i) - min_l;
    if (dist < 0)
      dist += mp_.map_voxel_num_(i);
    id_g(i) = mp_.map_bound_min_idx_(i) + dist;
  }
}

/// @brief 对指定膨胀层执行增量计数更新；ignore_mask 用于滑窗清理时避免重复影响待清理单元。
void GridMap::updateInflationLayer(const Eigen::Vector3i& id, int delta,
                                   const vector<Eigen::Vector3i>& offsets,
                                   std::vector<int>& cnt_buffer,
                                   std::vector<char>& flag_buffer,
                                   const std::vector<char>* ignore_mask)
{
  for (const auto& offset : offsets)
  {
    // id 是源障碍体素，offset 是预计算的圆柱膨胀偏移；inf_id 是需要更新的“禁止中心进入”的格。
    const Eigen::Vector3i inf_id = id + offset;
    if (!isInMap(inf_id))
      continue;

    const int addr = toAddress(inf_id);
    if (ignore_mask && (*ignore_mask)[addr])
      continue;

    cnt_buffer[addr] += delta;
    if (cnt_buffer[addr] < 0)
      cnt_buffer[addr] = 0;
    // 这里提前写好膨胀标志；之后 getInflateOccupancyFromBuffer() 检查圆柱中心时直接读取它。
    flag_buffer[addr] = cnt_buffer[addr] > 0 ? 1 : 0;
  }
}

/// @brief 将一个源体素的占用增减传给膨胀更新；源单元共享膨胀邻域时依赖计数维护结果。
void GridMap::updateInflation(const Eigen::Vector3i& id, int delta, const std::vector<char>* ignore_mask)
{
  updateInflationLayer(id, delta, md_.inflate_offsets_, md_.occupancy_buffer_inflate_cnt_,
                       md_.occupancy_buffer_inflate_, ignore_mask);
}

/// @brief 写入新的 log-odds，仅在跨越占用判定阈值时增减其膨胀贡献。
void GridMap::applyOccupancyUpdate(const Eigen::Vector3i& id, double new_log_odds)
{
  if (!isInMap(id))
    return;

  const int addr = toAddress(id);
  const bool was_occ = md_.occupancy_buffer_[addr] > mp_.min_occupancy_log_;
  const bool now_occ = new_log_odds > mp_.min_occupancy_log_;

  md_.occupancy_buffer_[addr] = new_log_odds;
  if (was_occ != now_occ)
    updateInflation(id, now_occ ? 1 : -1);
}

/// @brief 清理一个缓冲单元并撤销它原有的膨胀贡献，恢复观测计数及射线去重标记。
void GridMap::resetCellByAddress(int addr)
{
  Eigen::Vector3i id_g;
  hashIdToGlobalIndex(addr, id_g);
  if (md_.occupancy_buffer_[addr] > mp_.min_occupancy_log_)
    updateInflation(id_g, -1);

  md_.occupancy_buffer_[addr] = mp_.clamp_min_log_ - mp_.unknown_flag_;
  md_.count_hit_[addr] = 0;
  md_.count_hit_and_miss_[addr] = 0;
  md_.flag_rayend_[addr] = -1;
  md_.flag_traverse_[addr] = -1;
}

/// @brief 滑动窗口的单元清理版本，结合 clear_mask 避免对即将清除的单元重复更新。
void GridMap::resetCellByAddressForSliding(int addr, const std::vector<char>& clear_mask)
{
  Eigen::Vector3i id_g;
  hashIdToGlobalIndex(addr, id_g);
  if (md_.occupancy_buffer_[addr] > mp_.min_occupancy_log_)
    updateInflation(id_g, -1, &clear_mask);
}

/// @brief 按传感器中心体素索引的位移移动窗口；小位移保留旧数据并清理退出区域，大跨度则重置。
/// 移动的是逻辑窗口边界，固定内存通过取模地址复用，不能按普通数组平移理解。
void GridMap::updateSlidingMap(const Eigen::Vector3d& center)
{
  // 设计目的：随传感器位置移动精细局部地图，为当前搜索/优化/安全检查保留附近障碍信息，并让存储容量有界。
  // 正常滑动时重叠体素的世界索引及取模地址不变，无需搬移整块数据；移出区域的地址清理后复用于新区域。
  // 新区域按未观测状态重置，随后由传感器融合填充；滑动不是把新进入区域自动判成已知自由空间。
  if (!mp_.map_sliding_en_)
    return;

  Eigen::Vector3i new_origin_idx;
  posToIndex(center, new_origin_idx);
  // 按全局体素中心位移决定窗口是否移动；取模地址复用需要先清理退出区域的旧占用贡献。
  const Eigen::Vector3i shift_num = new_origin_idx - mp_.map_origin_idx_;
  if (shift_num.cwiseAbs().maxCoeff() < mp_.map_sliding_thresh_vox_)
    return;

  if ((shift_num.cwiseAbs().array() >= mp_.map_voxel_num_.array()).any())
  {
    resetAllMapData();
    mp_.map_origin_idx_ = new_origin_idx;
    updateMapBoundaryFromIndex();
    md_.local_bound_min_ = mp_.map_bound_min_idx_;
    md_.local_bound_max_ = mp_.map_bound_max_idx_;
    return;
  }

  const int buffer_size = mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2);
  // 当前实现仍初始化一个窗口大小的临时掩码，并处理退出体素/膨胀贡献；“重叠数据免拷贝”不等于整个更新严格 O(1)。
  std::vector<char> clear_mask(buffer_size, 0);
  std::vector<int> clear_addrs;
  clear_addrs.reserve(buffer_size / 8);

  auto add_clear_addr = [&](const Eigen::Vector3i& id_l) {
    const int addr = toAddressLocal(id_l);
    if (!clear_mask[addr])
    {
      clear_mask[addr] = 1;
      clear_addrs.push_back(addr);
    }
  };

  for (int dim = 0; dim < 3; ++dim)
  {
    const int shift = shift_num(dim);
    if (shift == 0)
      continue;

    if (shift > 0)
    {
      for (int k = 0; k < shift; ++k)
      {
        const int clear_g = mp_.map_bound_min_idx_(dim) + k;
        const int clear_l = getLocalIndex(clear_g, dim);

        for (int a = 0; a < mp_.map_voxel_num_((dim + 1) % 3); ++a)
          for (int b = 0; b < mp_.map_voxel_num_((dim + 2) % 3); ++b)
          {
            Eigen::Vector3i id_l;
            id_l(dim) = clear_l;
            id_l((dim + 1) % 3) = a;
            id_l((dim + 2) % 3) = b;
            add_clear_addr(id_l);
          }
      }
    }
    else
    {
      for (int k = 0; k < -shift; ++k)
      {
        const int clear_g = mp_.map_bound_max_idx_(dim) - k;
        const int clear_l = getLocalIndex(clear_g, dim);

        for (int a = 0; a < mp_.map_voxel_num_((dim + 1) % 3); ++a)
          for (int b = 0; b < mp_.map_voxel_num_((dim + 2) % 3); ++b)
          {
            Eigen::Vector3i id_l;
            id_l(dim) = clear_l;
            id_l((dim + 1) % 3) = a;
            id_l((dim + 2) % 3) = b;
            add_clear_addr(id_l);
          }
      }
    }
  }

  for (int addr : clear_addrs)
    resetCellByAddressForSliding(addr, clear_mask);

  for (int addr : clear_addrs)
  {
    md_.occupancy_buffer_[addr] = mp_.clamp_min_log_ - mp_.unknown_flag_;
    md_.occupancy_buffer_inflate_cnt_[addr] = 0;
    md_.occupancy_buffer_inflate_[addr] = 0;
    md_.count_hit_[addr] = 0;
    md_.count_hit_and_miss_[addr] = 0;
    md_.flag_rayend_[addr] = -1;
    md_.flag_traverse_[addr] = -1;
  }

  mp_.map_origin_idx_ = new_origin_idx;
  updateMapBoundaryFromIndex();
  boundIndex(md_.local_bound_min_);
  boundIndex(md_.local_bound_max_);
}

/// @brief 清空当前地图窗口中的缓冲数据；区域版本可用于限定清理范围。
void GridMap::resetBuffer()
{
  resetAllMapData();
  md_.local_bound_min_ = mp_.map_bound_min_idx_;
  md_.local_bound_max_ = mp_.map_bound_max_idx_;
}

/// @brief 按世界坐标包围盒清理指定区域，边界转换到体素后逐单元撤销占用和膨胀贡献。
void GridMap::resetBuffer(Eigen::Vector3d min_pos, Eigen::Vector3d max_pos)
{
  Eigen::Vector3i min_id, max_id;
  posToIndex(min_pos, min_id);
  posToIndex(max_pos, max_id);

  boundIndex(min_id);
  boundIndex(max_id);

  for (int x = min_id(0); x <= max_id(0); ++x)
    for (int y = min_id(1); y <= max_id(1); ++y)
    {
      for (int z = min_id(2); z <= max_id(2); ++z)
      {
        resetCellByAddress(toAddress(x, y, z));
      }
    }
}

/// @brief 缓存当前观测对体素的 hit/miss 证据，返回地址；不在这里立即完成整轮概率融合。
int GridMap::setCacheOccupancy(Eigen::Vector3d pos, int occ)
{
  if (occ != 1 && occ != 0)
    return INVALID_IDX;

  Eigen::Vector3i id;
  posToIndex(pos, id);
  if (!isInMap(id))
    return INVALID_IDX;

  int idx_ctns = toAddress(id);

  md_.count_hit_and_miss_[idx_ctns] += 1;

  if (md_.count_hit_and_miss_[idx_ctns] == 1)
  {
    md_.cache_voxel_.push(id);
  }

  if (occ == 1)
    md_.count_hit_[idx_ctns] += 1;

  return idx_ctns;
}

/// @brief 按内参反投影稀疏深度像素并转换到世界坐标。
/// 太近像素跳过；零值/过远值生成超量程射线，后续用于自由空间证据而非真实障碍端点。
void GridMap::projectDepthImage()
{
  // md_.proj_points_.clear();
  md_.proj_points_cnt = 0;

  uint16_t *row_ptr;
  // int cols = current_img_.cols, rows = current_img_.rows;
  int cols = md_.depth_image_.cols;
  int rows = md_.depth_image_.rows;

  double depth;

  Eigen::Matrix3d sensor_r = md_.ray_q_.toRotationMatrix();

  // cout << "rotate: " << md_.ray_q_.toRotationMatrix() << endl;
  // std::cout << "pos in proj: " << md_.ray_pos_ << std::endl;

  if (!md_.has_first_depth_)
  {
    md_.has_first_depth_ = true;
    return;
  }

  Eigen::Vector3d pt_cur, pt_world;
  const double inv_factor = 1.0 / mp_.k_depth_scaling_factor_;

  // margin 是图像边框厚度（像素），与地图分辨率无关；在内缩后的范围里再按 skip_pixel 稀疏取样。
  for (int v = mp_.depth_filter_margin_; v < rows - mp_.depth_filter_margin_; v += mp_.skip_pixel_)
  {
    row_ptr = md_.depth_image_.ptr<uint16_t>(v) + mp_.depth_filter_margin_;

    for (int u = mp_.depth_filter_margin_; u < cols - mp_.depth_filter_margin_; u += mp_.skip_pixel_)
    {
      const uint16_t raw_depth = *row_ptr;
      depth = raw_depth * inv_factor;
      row_ptr = row_ptr + mp_.skip_pixel_;

      // filter depth
      // depth += rand_noise_(eng_);
      // if (depth > 0.01) depth += rand_noise2_(eng_);

      if (raw_depth == 0)
      {
        // 现有观测策略：无返回值也按超量程方向提供 miss 证据，不代表零值能证明前方实际空闲。
        depth = mp_.max_ray_length_ + 0.1;
      }
      else if (depth < mp_.depth_filter_mindist_)
      {
        // 太近的非零测量直接丢弃，不是将 0.3 m 内地图全部清空；0.05 m 体素边长不参与此比较。
        continue;
      }
      else if (depth > mp_.depth_filter_maxdist_)
      {
        // 这里设置的是相机 Z 深度；反投影后的欧氏射线长度至少为它，后续由 raycastProcess 截断并记 miss。
        depth = mp_.max_ray_length_ + 0.1;
      }

      // project to world frame
      pt_cur(0) = (u - mp_.cx_) * depth / mp_.fx_;
      pt_cur(1) = (v - mp_.cy_) * depth / mp_.fy_;
      pt_cur(2) = depth;

      pt_world = sensor_r * pt_cur + md_.ray_pos_;
      // if (!isInMap(pt_world)) {
      //   pt_world = closetPointInMap(pt_world, md_.ray_pos_);
      // }

      md_.proj_points_[md_.proj_points_cnt++] = pt_world;
    }
  }
}

/// @brief 将观测端点和射线穿过单元分别计作命中/未命中，按缓存证据选择更新并截断 log-odds。
/// 射线限制长度和局部范围，并用标记避免本轮重复处理；占用跨阈值时联动膨胀层。
void GridMap::raycastProcess()
{
  // if (md_.proj_points_.size() == 0)
  if (md_.proj_points_cnt == 0)
    return;

  updateSlidingMap(md_.ray_pos_);

  ros::Time t1, t2;

  md_.raycast_num_ += 1;

  int vox_idx;
  double length;

  // bounding box of updated region
  double min_x = mp_.map_max_boundary_(0);
  double min_y = mp_.map_max_boundary_(1);
  double min_z = mp_.map_max_boundary_(2);

  double max_x = mp_.map_min_boundary_(0);
  double max_y = mp_.map_min_boundary_(1);
  double max_z = mp_.map_min_boundary_(2);

  RayCaster raycaster;
  Eigen::Vector3d half = Eigen::Vector3d(0.5, 0.5, 0.5);
  Eigen::Vector3d ray_pt, pt_w;

  for (int i = 0; i < md_.proj_points_cnt; ++i)
  {
    pt_w = md_.proj_points_[i];

    // set flag for projected point

    if (!isInMap(pt_w))
    {
      pt_w = closetPointInMap(pt_w, md_.ray_pos_);

      length = (pt_w - md_.ray_pos_).norm();
      if (length > mp_.max_ray_length_)
      {
        pt_w = (pt_w - md_.ray_pos_) / length * mp_.max_ray_length_ + md_.ray_pos_;
      }
      vox_idx = setCacheOccupancy(pt_w, 0);
    }
    else
    {
      length = (pt_w - md_.ray_pos_).norm();

      if (length > mp_.max_ray_length_)
      {
        pt_w = (pt_w - md_.ray_pos_) / length * mp_.max_ray_length_ + md_.ray_pos_;
        vox_idx = setCacheOccupancy(pt_w, 0);
      }
      else
      {
        vox_idx = setCacheOccupancy(pt_w, 1);
      }
    }

    max_x = max(max_x, pt_w(0));
    max_y = max(max_y, pt_w(1));
    max_z = max(max_z, pt_w(2));

    min_x = min(min_x, pt_w(0));
    min_y = min(min_y, pt_w(1));
    min_z = min(min_z, pt_w(2));

    // raycasting between ray origin and point

    if (vox_idx != INVALID_IDX)
    {
      if (md_.flag_rayend_[vox_idx] == md_.raycast_num_)
      {
        continue;
      }
      else
      {
        md_.flag_rayend_[vox_idx] = md_.raycast_num_;
      }
    }

    raycaster.setInput(pt_w / mp_.resolution_, md_.ray_pos_ / mp_.resolution_);

    while (raycaster.step(ray_pt))
    {
      Eigen::Vector3d tmp = (ray_pt + half) * mp_.resolution_;
      length = (tmp - md_.ray_pos_).norm();

      vox_idx = setCacheOccupancy(tmp, 0);

      if (vox_idx != INVALID_IDX)
      {
        if (md_.flag_traverse_[vox_idx] == md_.raycast_num_)
        {
          break;
        }
        else
        {
          md_.flag_traverse_[vox_idx] = md_.raycast_num_;
        }
      }
    }
  }

  min_x = min(min_x, md_.ray_pos_(0));
  min_y = min(min_y, md_.ray_pos_(1));
  min_z = min(min_z, md_.ray_pos_(2));

  max_x = max(max_x, md_.ray_pos_(0));
  max_y = max(max_y, md_.ray_pos_(1));
  max_z = max(max_z, md_.ray_pos_(2));
  max_z = max(max_z, mp_.ground_height_);

  posToIndex(Eigen::Vector3d(max_x, max_y, max_z), md_.local_bound_max_);
  posToIndex(Eigen::Vector3d(min_x, min_y, min_z), md_.local_bound_min_);
  boundIndex(md_.local_bound_min_);
  boundIndex(md_.local_bound_max_);

  // update occupancy cached in queue
  Eigen::Vector3d local_range_min = md_.ray_pos_ - mp_.local_update_range_;
  Eigen::Vector3d local_range_max = md_.ray_pos_ + mp_.local_update_range_;

  Eigen::Vector3i min_id, max_id;
  posToIndex(local_range_min, min_id);
  posToIndex(local_range_max, max_id);
  boundIndex(min_id);
  boundIndex(max_id);

  // std::cout << "cache all: " << md_.cache_voxel_.size() << std::endl;

  while (!md_.cache_voxel_.empty())
  {

    Eigen::Vector3i idx = md_.cache_voxel_.front();
    int idx_ctns = toAddress(idx);
    md_.cache_voxel_.pop();

    double log_odds_update =
        md_.count_hit_[idx_ctns] >= md_.count_hit_and_miss_[idx_ctns] - md_.count_hit_[idx_ctns] ? mp_.prob_hit_log_ : mp_.prob_miss_log_;

    md_.count_hit_[idx_ctns] = md_.count_hit_and_miss_[idx_ctns] = 0;

    if (log_odds_update >= 0 && md_.occupancy_buffer_[idx_ctns] >= mp_.clamp_max_log_)
    {
      continue;
    }
    else if (log_odds_update <= 0 && md_.occupancy_buffer_[idx_ctns] <= mp_.clamp_min_log_)
    {
      applyOccupancyUpdate(idx, mp_.clamp_min_log_);
      continue;
    }

    bool in_local = idx(0) >= min_id(0) && idx(0) <= max_id(0) && idx(1) >= min_id(1) &&
                    idx(1) <= max_id(1) && idx(2) >= min_id(2) && idx(2) <= max_id(2);
    if (!in_local)
    {
      applyOccupancyUpdate(idx, mp_.clamp_min_log_);
    }

    // 本轮证据叠加后截断到概率上下界；阈值跨越由 applyOccupancyUpdate() 同步到膨胀计数。
    const double new_log_odds =
        std::min(std::max(md_.occupancy_buffer_[idx_ctns] + log_odds_update, mp_.clamp_min_log_),
                 mp_.clamp_max_log_);
    applyOccupancyUpdate(idx, new_log_odds);
  }
}

/// @brief 沿传感器到观测点的方向求地图边界内的截断点，供地图外观测的射线更新使用。
Eigen::Vector3d GridMap::closetPointInMap(const Eigen::Vector3d &pt, const Eigen::Vector3d &ray_pos)
{
  Eigen::Vector3d diff = pt - ray_pos;
  Eigen::Vector3d max_tc = mp_.map_max_boundary_ - ray_pos;
  Eigen::Vector3d min_tc = mp_.map_min_boundary_ - ray_pos;

  double min_t = 1000000;

  for (int i = 0; i < 3; ++i)
  {
    if (fabs(diff[i]) > 0)
    {

      double t1 = max_tc[i] / diff[i];
      if (t1 > 0 && t1 < min_t)
        min_t = t1;

      double t2 = min_tc[i] / diff[i];
      if (t2 > 0 && t2 < min_t)
        min_t = t2;
    }
  }

  return ray_pos + (min_t - 1e-3) * diff;
}

/// @brief 定时发布地图和窗口显示信息；这些显示裁剪不改变内部碰撞查询结果。
void GridMap::visCallback(const ros::TimerEvent & /*event*/)
{

  publishMap();
  publishMapInflate(true);
  publishSlidingMapFrame();
  publishSlidingMapBBox();
  publishDepthCloud();
}

/// @brief 仅在有待融合观测时进行更新；深度先反投影，点云已由接收回调准备，再统一射线融合。
void GridMap::updateOccupancyCallback(const ros::TimerEvent & /*event*/)
{
  if (!md_.occ_need_update_)
    return;

  /* update occupancy */
  // ros::Time t1, t2, t3, t4;
  // t1 = ros::Time::now();

  if (!md_.use_cloud_update_)
    projectDepthImage();
  // t2 = ros::Time::now();
  raycastProcess();
  // t3 = ros::Time::now();

  // t4 = ros::Time::now();

  // cout << setprecision(7);
  // cout << "t2=" << (t2-t1).toSec() << " t3=" << (t3-t2).toSec() << " t4=" << (t4-t3).toSec() << endl;;

  // md_.fuse_time_ += (t2 - t1).toSec();
  // md_.max_fuse_time_ = max(md_.max_fuse_time_, (t2 - t1).toSec());

  // if (mp_.show_occ_time_)
  //   ROS_WARN("Fusion: cur t = %lf, avg t = %lf, max t = %lf", (t2 - t1).toSec(),
  //            md_.fuse_time_ / md_.update_num_, md_.max_fuse_time_);

  md_.occ_need_update_ = false;
  md_.use_cloud_update_ = false;
}

/// @brief 同步接收深度图和位姿，必要时转换深度编码并补传感器外参，标记一次地图融合请求。
void GridMap::depthPoseCallback(const sensor_msgs::ImageConstPtr &img,
                                const nav_msgs::OdometryConstPtr &pose)
{
  if (mp_.sensor_type_ != "depth")
    return;

  /* get depth image */
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(img, img->encoding);

  if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
  {
    (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, mp_.k_depth_scaling_factor_);
  }
  cv_ptr->image.copyTo(md_.depth_image_);

  // std::cout << "depth: " << md_.depth_image_.cols << ", " << md_.depth_image_.rows << std::endl;

  /* get pose */
  const geometry_msgs::Pose &sensor_pose = pose->pose.pose;
  Eigen::Quaterniond ray_q(sensor_pose.orientation.w, sensor_pose.orientation.x,
                           sensor_pose.orientation.y, sensor_pose.orientation.z);
  if (ray_q.norm() < 1e-6)
    return;
  ray_q.normalize();

  Eigen::Vector3d ray_pos(sensor_pose.position.x, sensor_pose.position.y, sensor_pose.position.z);
  if (mp_.need_extrinsic_)
  {
    const Eigen::Matrix3d pose_r = ray_q.toRotationMatrix();
    ray_pos += pose_r * mp_.depth_extrinsic_.block<3, 1>(0, 3);
    ray_q = Eigen::Quaterniond(pose_r * mp_.depth_extrinsic_.block<3, 3>(0, 0));
    ray_q.normalize();
  }

  nav_msgs::Odometry extrinsic_pose = *pose;
  extrinsic_pose.pose.pose.position.x = ray_pos.x();
  extrinsic_pose.pose.pose.position.y = ray_pos.y();
  extrinsic_pose.pose.pose.position.z = ray_pos.z();
  extrinsic_pose.pose.pose.orientation.x = ray_q.x();
  extrinsic_pose.pose.pose.orientation.y = ray_q.y();
  extrinsic_pose.pose.pose.orientation.z = ray_q.z();
  extrinsic_pose.pose.pose.orientation.w = ray_q.w();
  extrinsic_pose.child_frame_id =
      pose->child_frame_id.empty() ? "sensor_extrinsic" : pose->child_frame_id + "_extrinsic";
  extrinsic_pose_pub_.publish(extrinsic_pose);

  md_.ray_pos_ = ray_pos;
  md_.ray_q_ = ray_q;
  md_.use_cloud_update_ = false;
  updateSlidingMap(md_.ray_pos_);
  if (isInMap(md_.ray_pos_))
  {
    md_.has_ray_pose_ = true;
    md_.update_num_ += 1;
    md_.occ_need_update_ = true;
  }
  else
  {
    md_.occ_need_update_ = false;
  }
}

/// @brief lidar 位姿入口：归一化四元数、按需应用外参并保存射线原点，同时更新滑动窗口。
// 默认仿真 /grid_map/sensor_pose 重映射到 /quad_0/lidar_pose，由 pcl_render_node 组合机身位姿与雷达安装方向发布。
// cloud_is_world=true 时不再用 ray_q_ 旋转点云 XYZ，但 ray_pos_ 仍用于射线起点/滑窗；真实安装偏移不能凭话题改名省略。
void GridMap::sensorPoseCallback(const nav_msgs::OdometryConstPtr &pose_msg)
{
  if (mp_.sensor_type_ != "lidar")
    return;

  const geometry_msgs::Pose &sensor_pose = pose_msg->pose.pose;
  Eigen::Quaterniond ray_q(sensor_pose.orientation.w, sensor_pose.orientation.x,
                           sensor_pose.orientation.y, sensor_pose.orientation.z);
  if (ray_q.norm() < 1e-6)
    return;
  ray_q.normalize();

  Eigen::Vector3d ray_pos(sensor_pose.position.x, sensor_pose.position.y, sensor_pose.position.z);
  if (mp_.need_extrinsic_)
  {
    const Eigen::Matrix3d pose_r = ray_q.toRotationMatrix();
    ray_pos += pose_r * mp_.lidar_extrinsic_.block<3, 1>(0, 3);
    ray_q = Eigen::Quaterniond(pose_r * mp_.lidar_extrinsic_.block<3, 3>(0, 0));
    ray_q.normalize();
  }
  if (!std::isfinite(ray_pos.x()) || !std::isfinite(ray_pos.y()) || !std::isfinite(ray_pos.z()))
    return;

  md_.ray_pos_ = ray_pos;
  md_.ray_q_ = ray_q;
  md_.has_ray_pose_ = true;
  updateSlidingMap(md_.ray_pos_);
}

/// @brief 记录机身平移位置，供 RViz 的 sliding_map 随动坐标系使用；不替代传感器射线位姿。
void GridMap::slidingMapFrameCallback(const nav_msgs::OdometryConstPtr &pose)
{
  const geometry_msgs::Point &pos = pose->pose.pose.position;
  md_.sliding_map_frame_pos_ = Eigen::Vector3d(pos.x, pos.y, pos.z);
}

/// @brief lidar 点云入口：跳过无位姿或非有限数据，按 cloud_is_world 决定是否转换 XYZ，再准备融合。
void GridMap::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &img)
{
  if (mp_.sensor_type_ != "lidar")
    return;

  if (!md_.has_ray_pose_)
  {
    ROS_WARN_THROTTLE(1.0, "[GridMap] no /grid_map/sensor_pose yet for lidar cloud update.");
    return;
  }

  // 规划建图只读取 XYZ；强度在当前占用更新中没有作为额外概率权重。
  pcl::PointCloud<pcl::PointXYZ> latest_cloud;
  pcl::fromROSMsg(*img, latest_cloud);

  md_.has_cloud_ = true;

  if (latest_cloud.points.size() == 0)
    return;

  const Eigen::Matrix3d sensor_r = md_.ray_q_.toRotationMatrix();
  const Eigen::Vector3d ray_pos = md_.ray_pos_;
  if (!std::isfinite(ray_pos.x()) || !std::isfinite(ray_pos.y()) || !std::isfinite(ray_pos.z()))
    return;

  updateSlidingMap(ray_pos);

  md_.proj_points_cnt = 0;

  for (size_t i = 0; i < latest_cloud.points.size(); ++i)
  {
    const pcl::PointXYZ &pt = latest_cloud.points[i];
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z))
      continue;

    Eigen::Vector3d pt_world;
    if (mp_.cloud_is_world_)
    {
      pt_world = Eigen::Vector3d(pt.x, pt.y, pt.z);
    }
    else
    {
      const Eigen::Vector3d pt_sensor(pt.x, pt.y, pt.z);
      pt_world = sensor_r * pt_sensor + ray_pos;
    }
    const Eigen::Vector3d devi = pt_world - ray_pos;
    const double ray_length = devi.norm();
    const bool in_local_range =
        fabs(devi(0)) <= mp_.local_update_range_(0) && fabs(devi(1)) <= mp_.local_update_range_(1) &&
        fabs(devi(2)) <= mp_.local_update_range_(2);
    if (!in_local_range && ray_length <= mp_.max_ray_length_)
      continue;

    if (md_.proj_points_cnt >= static_cast<int>(md_.proj_points_.size()))
      md_.proj_points_.push_back(pt_world);
    else
      md_.proj_points_[md_.proj_points_cnt] = pt_world;

    md_.proj_points_cnt++;
  }

  if (md_.proj_points_cnt == 0)
    return;

  md_.use_cloud_update_ = true;
  md_.occ_need_update_ = true;
}

/// @brief 发布已占用体素点云；超过传感器 z 加 vis_height 的点不显示，内部占用仍保留。
void GridMap::publishMap()
{

  if (map_pub_.getNumSubscribers() <= 0)
    return;

  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector3i min_cut = mp_.map_bound_min_idx_;
  Eigen::Vector3i max_cut = mp_.map_bound_max_idx_;

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z)
      {
        if (md_.occupancy_buffer_[toAddress(x, y, z)] < mp_.min_occupancy_log_)
          continue;

        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);
        if (md_.has_ray_pose_ && pos(2) > md_.ray_pos_(2) + mp_.vis_height_)
          continue;
        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        cloud.push_back(pt);
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;

  pcl::toROSMsg(cloud, cloud_msg);
  map_pub_.publish(cloud_msg);
}

/// @brief 发布膨胀占用体素，用于检查机器人碰撞几何与环境裕量的关系。
// 输出各膨胀格中心的点云，不输出圆柱 Marker；default.rviz 用 0.1 m 的 Flat Squares 显示，外观不等于体素实体边界。
void GridMap::publishMapInflate(bool all_info)
{

  if (map_inf_pub_.getNumSubscribers() <= 0)
    return;

  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector3i min_cut = mp_.map_bound_min_idx_;
  Eigen::Vector3i max_cut = mp_.map_bound_max_idx_;

  const std::vector<char> &inflate_buffer = md_.occupancy_buffer_inflate_;
  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z)
      {
        if (inflate_buffer[toAddress(x, y, z)] == 0)
          continue;

        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);
        if (md_.has_ray_pose_ && pos(2) > md_.ray_pos_(2) + mp_.vis_height_)
          continue;

        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        cloud.push_back(pt);
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;

  pcl::toROSMsg(cloud, cloud_msg);
  map_inf_pub_.publish(cloud_msg);

  // ROS_INFO("pub map");
}

/// @brief 发布世界系到 sliding_map 的 TF：平移跟随机身、旋转为单位旋转，供显示视角跟随。
void GridMap::publishSlidingMapFrame()
{
  if (mp_.sliding_map_frame_id_.empty())
    return;

  tf::Transform transform;
  transform.setOrigin(tf::Vector3(md_.sliding_map_frame_pos_.x(), md_.sliding_map_frame_pos_.y(),
                                  md_.sliding_map_frame_pos_.z()));
  transform.setRotation(tf::Quaternion::getIdentity());
  tf_broadcaster_->sendTransform(
      tf::StampedTransform(transform, ros::Time::now(), mp_.frame_id_, mp_.sliding_map_frame_id_));
}

/// @brief 把当前逻辑窗口边界转换为线框 Marker，便于检查窗口是否随位置更新。
void GridMap::publishSlidingMapBBox()
{
  if (sliding_map_bbox_pub_.getNumSubscribers() <= 0)
    return;

  visualization_msgs::Marker marker;
  marker.header.frame_id = mp_.frame_id_;
  marker.header.stamp = ros::Time::now();
  marker.ns = "sliding_map";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.04;
  marker.color.r = 0.0;
  marker.color.g = 0.8;
  marker.color.b = 1.0;
  marker.color.a = 1.0;

  const Eigen::Vector3d& min_pt = mp_.map_min_boundary_;
  const Eigen::Vector3d& max_pt = mp_.map_max_boundary_;
  Eigen::Vector3d corners[8] = {
      {min_pt.x(), min_pt.y(), min_pt.z()},
      {max_pt.x(), min_pt.y(), min_pt.z()},
      {max_pt.x(), max_pt.y(), min_pt.z()},
      {min_pt.x(), max_pt.y(), min_pt.z()},
      {min_pt.x(), min_pt.y(), max_pt.z()},
      {max_pt.x(), min_pt.y(), max_pt.z()},
      {max_pt.x(), max_pt.y(), max_pt.z()},
      {min_pt.x(), max_pt.y(), max_pt.z()},
  };

  auto pushPoint = [&](const Eigen::Vector3d& p) {
    geometry_msgs::Point point;
    point.x = p.x();
    point.y = p.y();
    point.z = p.z();
    marker.points.push_back(point);
  };

  const int edges[12][2] = {
      {0, 1}, {1, 2}, {2, 3}, {3, 0},
      {4, 5}, {5, 6}, {6, 7}, {7, 4},
      {0, 4}, {1, 5}, {2, 6}, {3, 7},
  };

  for (const auto& edge : edges)
  {
    pushPoint(corners[edge[0]]);
    pushPoint(corners[edge[1]]);
  }

  sliding_map_bbox_pub_.publish(marker);
}

/// @brief 发布按内部未知标记筛选出的体素；未知状态与越出窗口范围需分别理解。
void GridMap::publishUnknown()
{
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector3i min_cut = md_.local_bound_min_;
  Eigen::Vector3i max_cut = md_.local_bound_max_;

  boundIndex(max_cut);
  boundIndex(min_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z)
      {

        if (md_.occupancy_buffer_[toAddress(x, y, z)] < mp_.clamp_min_log_ - 1e-3)
        {
          Eigen::Vector3d pos;
          indexToPos(Eigen::Vector3i(x, y, z), pos);
          if (md_.has_ray_pose_ && pos(2) > md_.ray_pos_(2) + mp_.vis_height_)
            continue;

          pt.x = pos(0);
          pt.y = pos(1);
          pt.z = pos(2);
          cloud.push_back(pt);
        }
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  unknown_pub_.publish(cloud_msg);
}

/// @brief 报告是否已有可用射线位姿，而不是查询整个 ROS 定位链路的健康状态。
bool GridMap::odomValid() { return md_.has_ray_pose_; }

/// @brief 报告是否已执行首次深度观测处理。
bool GridMap::hasDepthObservation() { return md_.has_first_depth_; }

/// @brief 返回当前局部地图原点，单位 m。
Eigen::Vector3d GridMap::getOrigin() { return mp_.map_origin_; }

// int GridMap::getVoxelNum() {
//   return mp_.map_voxel_num_[0] * mp_.map_voxel_num_[1] * mp_.map_voxel_num_[2];
// }

/// @brief 返回当前窗口的世界系原点和尺寸，供其他模块限制搜索范围。
void GridMap::getRegion(Eigen::Vector3d &ori, Eigen::Vector3d &size)
{
  ori = mp_.map_origin_, size = mp_.map_size_;
}

// GridMap

/// @brief 将当前投影点发布为调试点云，用于核对深度反投影与传感器外参。
void GridMap::publishDepthCloud()
{
  if (depth_cloud_pub_.getNumSubscribers() <= 0)
    return;

  if (md_.proj_points_cnt == 0)
    return;

  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PointXYZ pt;

  for (int i = 0; i < md_.proj_points_cnt; ++i)
  {
    pt.x = md_.proj_points_[i](0);
    pt.y = md_.proj_points_[i](1);
    pt.z = md_.proj_points_[i](2);
    cloud.push_back(pt);
  }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  cloud_msg.header.stamp = ros::Time::now();
  depth_cloud_pub_.publish(cloud_msg);
}
