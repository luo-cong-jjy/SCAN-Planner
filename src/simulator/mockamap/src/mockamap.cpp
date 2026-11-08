/// @file
/// 随机/噪声/迷宫场景发布入口，默认 run.launch 通过 simulator.xml 启动它。
/// 先由 Maps::generate(type) 构造场景，再按 update_freq 重复发布同一份 PointCloud2；不融合传感器。
/// 发布原话题 mock_map 在 launch 中重映射成 /map_generator/global_cloud。
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <algorithm>
#include <iostream>
#include <vector>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include "maps.hpp"

void
optimizeMap(mocka::Maps::BasicInfo& in)
{
  std::vector<int>* temp = new std::vector<int>;

  pcl::KdTreeFLANN<pcl::PointXYZ>     kdtree;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  cloud->width  = in.cloud->width;
  cloud->height = in.cloud->height;
  cloud->points.resize(cloud->width * cloud->height);

  for (uint32_t i = 0; i < cloud->width; i++)
  {
    cloud->points[i].x = in.cloud->points[i].x;
    cloud->points[i].y = in.cloud->points[i].y;
    cloud->points[i].z = in.cloud->points[i].z;
  }

  kdtree.setInputCloud(cloud);
  double radius = 1.75 / in.scale; // 1.75 is the rounded up value of sqrt(3)

  for (uint32_t i = 0; i < cloud->width; i++)
  {
    std::vector<int>   pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    if (kdtree.radiusSearch(cloud->points[i], radius, pointIdxRadiusSearch,
                            pointRadiusSquaredDistance) >= 27)
    {
      temp->push_back(i);
    }
  }
  for (int i = temp->size() - 1; i >= 0; i--)
  {
    in.cloud->points.erase(in.cloud->points.begin() +
                           temp->at(i)); // erasing the enclosed points
  }
  in.cloud->width -= temp->size();

  pcl::toROSMsg(*in.cloud, *in.output);
  in.output->header.frame_id = "world";
  ROS_INFO("finish: number of points after optimization %d", in.cloud->width);
  delete temp;
  return;
}

// 这是 mockamap_node 地图发布程序的入口；规划器、控制器等其他可执行程序各有自己的 main()。
// 本程序读取场景配置，调用 Maps 生成环境点云，再周期发布给模拟感知与显示节点。
int
main(int argc, char** argv)
{
  ros::init(argc, argv, "mockamap");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  ros::Publisher pcl_pub =
    // 运行连接（输出入口）：随机场景输出，经 simulator.xml 重映射为 /map_generator/global_cloud，供
    // 感知节点渲染。
    // 队列容量：1 条消息，不是发布/规划频率。
    nh.advertise<sensor_msgs::PointCloud2>("mock_map", 1);
  pcl::PointCloud<pcl::PointXYZ> cloud;
  sensor_msgs::PointCloud2       output;
  // Fill in the cloud data

  int seed;

  int sizeX;
  int sizeY;
  int sizeZ;

  double scale;
  double update_freq;

  int type;

  // 本段读取启动配置：下方“正常默认”来自当前 launch/XML，实际运行以参数服务器为准。
  // param() 的末项仅是缺参回退；getParam()/必需参数封装没有这一回退，具体见各行说明。
  // 参数读取：固定随机种子，配合同一生成算法和配置使场景便于重复比较。
  // 配置来源：simulator.xml → /mockamap_node/seed；正常默认启动取 127。
  // 缺参回退：4546；只有参数服务器中没有该键时才使用。
  nh_private.param("seed", seed, 4546);
  // 参数读取：随机地图的重复发布频率，单位 Hz；场景在进入循环前生成一次，不是每轮重随机。
  // 配置来源：simulator.xml → /mockamap_node/update_freq；正常默认启动取 0.5。
  // 缺参回退：1.0；只有参数服务器中没有该键时才使用。
  nh_private.param("update_freq", update_freq, 1.0);
  // 参数读取：地图生成的基础离散长度，单位 m；生成器内部换算为 scale=1/resolution。
  // 配置来源：simulator.xml → /mockamap_node/resolution；正常默认启动取 0.1。
  // 缺参回退：0.38；只有参数服务器中没有该键时才使用。
  nh_private.param("resolution", scale, 0.38);
  // 参数读取：随机场景 x 向尺寸，单位 m，由生成器再转换为离散尺寸。
  // 配置来源：simulator.xml → /mockamap_node/x_length；正常默认启动取 40.0。
  // 缺参回退：100；只有参数服务器中没有该键时才使用。
  nh_private.param("x_length", sizeX, 100);
  // 参数读取：随机场景 y 向尺寸，单位 m。
  // 配置来源：simulator.xml → /mockamap_node/y_length；正常默认启动取 40.0。
  // 缺参回退：100；只有参数服务器中没有该键时才使用。
  nh_private.param("y_length", sizeY, 100);
  // 参数读取：随机场景 z 向尺寸，单位 m；具体障碍高度还由对应场景分支决定。
  // 配置来源：simulator.xml → /mockamap_node/z_length；正常默认启动取 5.0。
  // 缺参回退：10；只有参数服务器中没有该键时才使用。
  nh_private.param("z_length", sizeZ, 10);

  // 参数读取：场景类型：1=Perlin 噪声，2=随机箱体，3=二维迷宫，4=三维迷宫；默认执行 type 2。
  // 配置来源：simulator.xml → /mockamap_node/type；正常默认启动取 2。
  // 缺参回退：1；只有参数服务器中没有该键时才使用。
  nh_private.param("type", type, 1);

  /// 把米制分辨率换为每米格数，再把场景长度换到离散网格尺度；Maps 内部按 scale 换回米制坐标。
  scale = 1 / scale;
  sizeX = sizeX * scale;
  sizeY = sizeY * scale;
  sizeZ = sizeZ * scale;

  mocka::Maps::BasicInfo info;
  info.nh_private = &nh_private;
  info.sizeX      = sizeX;
  info.sizeY      = sizeY;
  info.sizeZ      = sizeZ;
  info.seed       = seed;
  info.scale      = scale;
  info.output     = &output;
  info.cloud      = &cloud;

  mocka::Maps map;
  map.setInfo(info);
  /// 只在进入发布循环前生成一次场景；重复发布不等于每帧重新随机生成。
  map.generate(type);

  //  optimizeMap(info);

  //! @note publish loop
  ros::Rate loop_rate(update_freq);
  while (ros::ok())
  {
    pcl_pub.publish(output);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
