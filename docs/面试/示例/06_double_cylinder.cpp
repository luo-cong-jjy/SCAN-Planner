// 平面截面手算：两圆形截面与一个点障碍；不包含地图体素、Z 厚度和未知区。
#include <cmath>
#include <iostream>

bool collides(double yaw) {
  constexpr double offset = 0.18;
  constexpr double radius = 0.25;
  constexpr double obstacle_x = 0.4;
  constexpr double obstacle_y = 0.0;
  for (double sign : {-1.0, 1.0}) {
    const double center_x = sign * offset * std::cos(yaw);
    const double center_y = sign * offset * std::sin(yaw);
    if (std::hypot(obstacle_x - center_x, obstacle_y - center_y) < radius)
      return true;
  }
  return false;
}

int main() {
  const double pi = std::acos(-1.0);
  std::cout << std::boolalpha
            << "yaw=0 collision=" << collides(0.0) << '\n'
            << "yaw=pi/2 collision=" << collides(pi / 2.0) << '\n';
  return collides(0.0) && !collides(pi / 2.0) ? 0 : 1;
}
