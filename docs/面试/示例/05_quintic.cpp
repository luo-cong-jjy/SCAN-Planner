// 单轴零首尾速度/加速度的五次多项式特例；不是项目 QR 求解器的替代实现。
#include <cmath>
#include <iostream>
#include <stdexcept>

struct State { double position, velocity, acceleration; };

State sample(double length, double duration, double time) {
  if (duration <= 0.0 || time < 0.0 || time > duration)
    throw std::invalid_argument("require positive duration and 0 <= time <= duration");
  const double u = time / duration;
  const double u2 = u * u;
  const double u3 = u2 * u;
  const double u4 = u3 * u;
  const double u5 = u4 * u;
  return {length * (10 * u3 - 15 * u4 + 6 * u5),
          length / duration * (30 * u2 - 60 * u3 + 30 * u4),
          length / (duration * duration) * (60 * u - 180 * u2 + 120 * u3)};
}

int main() {
  for (double time : {0.0, 1.0, 2.0}) {
    const auto state = sample(2.0, 2.0, time);
    std::cout << "t=" << time << ": p=" << state.position
              << ", v=" << state.velocity << ", a=" << state.acceleration << '\n';
  }
  const auto slow_midpoint = sample(2.0, 4.0, 2.0);
  std::cout << "T=4 midpoint velocity=" << slow_midpoint.velocity << '\n';
  // 验证六项边界值；不只验证某一个中点位置。
  const auto start = sample(2.0, 2.0, 0.0);
  const auto end = sample(2.0, 2.0, 2.0);
  if (std::abs(start.position) > 1e-10 || std::abs(start.velocity) > 1e-10 ||
      std::abs(start.acceleration) > 1e-10 || std::abs(end.position - 2.0) > 1e-10 ||
      std::abs(end.velocity) > 1e-10 || std::abs(end.acceleration) > 1e-10)
    return 1;
}
