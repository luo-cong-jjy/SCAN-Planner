/// @file
/// 射线经过体素的批量与增量接口；只处理几何遍历，不定义概率融合策略。
/// 调用者先把米制端点换到体素尺度；RayCaster 的步进输出由 GridMap 解释为自由空间证据。
#ifndef RAYCAST_H_
#define RAYCAST_H_

#include <Eigen/Eigen>
#include <vector>

double signum(double x);

double mod(double value, double modulus);

double intbound(double s, double ds);

void Raycast(const Eigen::Vector3d& start, const Eigen::Vector3d& end, const Eigen::Vector3d& min,
             const Eigen::Vector3d& max, int& output_points_cnt, Eigen::Vector3d* output);

void Raycast(const Eigen::Vector3d& start, const Eigen::Vector3d& end, const Eigen::Vector3d& min,
             const Eigen::Vector3d& max, std::vector<Eigen::Vector3d>* output);

/// @brief 保存各轴当前索引、步进方向、下一网格面参数距离和增量，支持不分配整段路径的逐格遍历。
class RayCaster {
private:
  /* data */
  Eigen::Vector3d start_;
  Eigen::Vector3d end_;
  Eigen::Vector3d direction_;
  Eigen::Vector3d min_;
  Eigen::Vector3d max_;
  int x_;
  int y_;
  int z_;
  int endX_;
  int endY_;
  int endZ_;
  double maxDist_;
  double dx_;
  double dy_;
  double dz_;
  int stepX_;
  int stepY_;
  int stepZ_;
  double tMaxX_;
  double tMaxY_;
  double tMaxZ_;
  double tDeltaX_;
  double tDeltaY_;
  double tDeltaZ_;
  double dist_;

  int step_num_;

public:
  RayCaster(/* args */) {
  }
  ~RayCaster() {
  }

  /// @brief 设置线段并初始化遍历状态；返回 false 表示退化线段，无需继续 step。
  bool setInput(const Eigen::Vector3d& start,
                const Eigen::Vector3d& end /* , const Eigen::Vector3d& min,
                const Eigen::Vector3d& max */);

  /// @brief 返回是否仍可输出下一遍历单元；终点是否作为 hit 由地图融合逻辑单独处理。
  bool step(Eigen::Vector3d& ray_pt);
};

#endif  // RAYCAST_H_