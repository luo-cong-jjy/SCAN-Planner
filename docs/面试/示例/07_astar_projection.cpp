// 只演示 A* 的参考高度投影公式，不包含开放集、障碍或完整搜索。
#include <algorithm>
#include <iostream>

int main() {
  const double start_x = 0.0, start_y = 0.0, start_z = 0.0;
  const double end_x = 4.0, end_y = 0.0, end_z = 2.0;
  const double dx = end_x - start_x, dy = end_y - start_y;
  const double length2 = dx * dx + dy * dy;

  // 按引用读取当前搜索的局部数据；lambda 只在这些局部量仍存活时调用。
  auto project_height = [&](double x, double y) {
    if (length2 < 1e-8) return start_z;
    double ratio = ((x - start_x) * dx + (y - start_y) * dy) / length2;
    ratio = std::max(0.0, std::min(1.0, ratio));
    return start_z + ratio * (end_z - start_z);
  };
  std::cout << "P=(2,3): z=" << project_height(2.0, 3.0) << '\n';
  std::cout << "P=(-1,3): z=" << project_height(-1.0, 3.0) << '\n';
  std::cout << "P=(6,3): z=" << project_height(6.0, 3.0) << '\n';
}
