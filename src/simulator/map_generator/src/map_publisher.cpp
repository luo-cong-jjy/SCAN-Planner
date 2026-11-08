/// @file
/// PCD 地图发布节点，simulator.xml 的 use_pcd_map 分支使用；默认随机地图不会运行本节点。
/// 读取 XYZ PCD、清理无效点、可选降采样和平移后发布静态场景点云，不执行定位或建图融合。
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <string>
#include <vector>

/// @brief 从命令行/私有参数选择文件，加载并预处理后按 publish_rate 重复发布到配置话题。
int main(int argc, char** argv) {
  ros::init(argc, argv, "map_pub");
  ros::NodeHandle node;
  ros::NodeHandle private_node("~");

  std::string file_name;
  // 文件路径可作为第一个程序参数传入，私有 file_name 参数随后可以覆盖它。
  if (argc > 1) {
    file_name = argv[1];
  }
  // 本段读取启动配置：下方“正常默认”来自当前 launch/XML，实际运行以参数服务器为准。
  // param() 的末项仅是缺参回退；getParam()/必需参数封装没有这一回退，具体见各行说明。
  // 参数读取：PCD 地图文件路径；可由命令行先提供，再由私有参数覆盖。
  // 配置位置：私有 /map_pub/file_name；当前主 launch 未覆盖此键，取下面的缺参回退。
  // 缺参回退：file_name；只有参数服务器中没有该键时才使用。
  private_node.param("file_name", file_name, file_name);

  std::string frame_id;
  std::string cloud_topic;
  double publish_rate;
  double downsample_res;
  double map_offset_x;
  double map_offset_y;
  double map_offset_z;
  // 参数读取：PCD 点云的参考坐标系名称，应与实际坐标和规划器期望一致。
  // 配置来源：simulator.xml → /map_pub/frame_id；该PCD 分支启用时取 world。
  // 缺参回退：std::string("world")；只有参数服务器中没有该键时才使用。
  private_node.param("frame_id", frame_id, std::string("world"));
  // 参数读取：PCD 地图统一输出话题，供感知渲染器和 RViz 使用。
  // 配置来源：simulator.xml → /map_pub/cloud_topic；该PCD 分支启用时取 /map_generator/global_cloud
  // 。
  // 缺参回退：std::string("/map_generator/global_cloud")；只有参数服务器中没有该键时才使用。
  private_node.param("cloud_topic", cloud_topic, std::string("/map_generator/global_cloud"));
  // 参数读取：PCD 静态地图重复发布频率，单位 Hz。
  // 配置来源：simulator.xml → /map_pub/publish_rate；该PCD 分支启用时取 0.2。
  // 缺参回退：3.0；只有参数服务器中没有该键时才使用。
  private_node.param("publish_rate", publish_rate, 3.0);
  // 参数读取：map_pub 的降采样长度尺度，单位 m；由上面的 downsample_resolution 传入。
  // 配置来源：simulator.xml → /map_pub/downsample_res；该PCD 分支启用时取 0.1。
  // 缺参回退：0.0；只有参数服务器中没有该键时才使用。
  private_node.param("downsample_res", downsample_res, 0.0);
  // 参数读取：发布 PCD 前施加的世界 x 平移（m），直接加到场景坐标。
  // 配置位置：私有 /map_pub/map_offset_x；当前主 launch 未覆盖此键，取下面的缺参回退。
  // 缺参回退：0.0；只有参数服务器中没有该键时才使用。
  private_node.param("map_offset_x", map_offset_x, 0.0);
  // 参数读取：发布 PCD 前施加的世界 y 平移（m）。
  // 配置位置：私有 /map_pub/map_offset_y；当前主 launch 未覆盖此键，取下面的缺参回退。
  // 缺参回退：0.0；只有参数服务器中没有该键时才使用。
  private_node.param("map_offset_y", map_offset_y, 0.0);
  // 参数读取：发布 PCD 前施加的世界 z 平移（m）。
  // 配置位置：私有 /map_pub/map_offset_z；当前主 launch 未覆盖此键，取下面的缺参回退。
  // 缺参回退：0.0；只有参数服务器中没有该键时才使用。
  private_node.param("map_offset_z", map_offset_z, 0.0);

  if (file_name.empty()) {
    ROS_ERROR("[map_pub] No PCD file specified. Pass it as an arg or set ~file_name.");
    return 1;
  }

  pcl::PointCloud<pcl::PointXYZ> cloud;
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, cloud) < 0) {
    ROS_ERROR_STREAM("[map_pub] Failed to read PCD file: " << file_name);
    return 1;
  }

  std::vector<int> finite_indices;
  pcl::removeNaNFromPointCloud(cloud, cloud, finite_indices);

  // 正的体素边长启用 PCL 降采样；零或负值保留原有有效点。
  if (downsample_res > 0.0) {
    pcl::VoxelGrid<pcl::PointXYZ> voxel_sampler;
    voxel_sampler.setInputCloud(cloud.makeShared());
    voxel_sampler.setLeafSize(downsample_res, downsample_res, downsample_res);
    voxel_sampler.filter(cloud);
  }

  // 将配置平移直接加到场景点坐标，长度单位 m；frame_id 名称本身不执行这个变换。
  if (map_offset_x != 0.0 || map_offset_y != 0.0 || map_offset_z != 0.0) {
    for (auto& point : cloud.points) {
      point.x += map_offset_x;
      point.y += map_offset_y;
      point.z += map_offset_z;
    }
  }

  if (cloud.empty()) {
    ROS_ERROR_STREAM("[map_pub] PCD file has no valid XYZ points: " << file_name);
    return 1;
  }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;

  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
  msg.header.frame_id = frame_id;

  ros::Publisher cloud_pub =
      // 运行连接（输出入口）：PCD 场景输出，私有 cloud_topic 缺省 /map_generator/global_cloud，供感
      // 知渲染与 RViz 消费。
      // 队列容量：10 条消息，不是发布/规划频率。 锁存最后一条消息，供新连接的订阅者立即取得。
      node.advertise<sensor_msgs::PointCloud2>(cloud_topic, 10, true);

  ROS_INFO_STREAM("[map_pub] Loaded " << cloud.points.size() << " points from " << file_name
                                      << ", publishing " << cloud_topic << " in frame "
                                      << frame_id);

  ros::Rate rate(std::max(0.1, publish_rate));
  while (ros::ok()) {
    msg.header.stamp = ros::Time::now();
    cloud_pub.publish(msg);
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
