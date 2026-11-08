// 独立的 C++14 练习，不加入规划器构建；每次先预测结果，再运行其中一组。
// 编译：g++ -std=c++14 -Wall -Wextra -pedantic tools/cpp_study_basics.cpp -o /tmp/scan_cpp_study
// 运行一组：/tmp/scan_cpp_study 1（数字可选 1～5）
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace cpp_study
{
void containers()
{
  std::vector<double> heights;
  heights.reserve(3);  // 预留容量，但元素数量仍是零。
  std::cout << "after reserve: size=" << heights.size() << '\n';
  heights.push_back(0.4);
  heights.push_back(0.6);
  heights.push_back(1.2);
  std::cout << "after append: size=" << heights.size()
            << ", first=" << heights.front() << ", last=" << heights.back() << '\n';
  heights.resize(5, 0.0);  // 新增两个元素，各自初始化为零。
  std::cout << "after resize: size=" << heights.size() << ", values:";
  for (const auto &height : heights)
    std::cout << ' ' << height;
  std::cout << '\n';
}

void changeCopy(std::vector<double> values)
{
  values.front() = 9.0;
}

void changeOriginal(std::vector<double> &values)
{
  values.front() = 9.0;
}

void showFirst(const std::vector<double> &values)
{
  std::cout << "first=" << values.front() << '\n';
}

void references()
{
  std::vector<double> heights{0.4, 1.2};
  changeCopy(heights);
  std::cout << "after value parameter: ";
  showFirst(heights);
  changeOriginal(heights);
  std::cout << "after reference parameter: ";
  showFirst(heights);
}

class Counter
{
public:
  explicit Counter(int initial) : value_(initial) {}  // 构造函数初始化成员。
  int increment();  // 在类中声明。

private:
  int value_;
};

int Counter::increment()  // 在类外定义。
{
  return ++value_;
}

void classes()
{
  Counter counter(0);  // 构造名为 counter 的对象。
  std::cout << "first call=" << counter.increment() << '\n';
  std::cout << "second call=" << counter.increment() << '\n';
}

void pointers()
{
  int value = 3;
  int *pointer = &value;  // 取得现有整数的地址。
  *pointer = 4;          // 通过指针修改该整数。
  std::cout << "value through raw pointer=" << value << '\n';

  std::unique_ptr<Counter> owner;
  owner.reset(new Counter(0));
  std::cout << "unique object call=" << owner->increment() << '\n';
  owner.reset();  // 销毁独占的对象，指针变为空。
  std::cout << "unique pointer is empty=" << std::boolalpha << !owner << '\n';

  auto shared = std::make_shared<Counter>(10);
  auto other_owner = shared;
  std::cout << "shared owners before reset=" << shared.use_count() << '\n';
  shared.reset();  // 另一个持有者仍保存着同一对象。
  std::cout << "remaining owners=" << other_owner.use_count()
            << ", shared object call=" << other_owner->increment() << '\n';
}

enum State
{
  INIT,
  WAIT_TARGET,
  GEN_NEW_TRAJ,
  REPLAN_TRAJ,
  EXEC_TRAJ,
  EMERGENCY_STOP
};

std::pair<int, State> readStatus()
{
  return std::pair<int, State>(2, GEN_NEW_TRAJ);
}

void enumsAndPairs()
{
  auto status = readStatus();  // 编译器推导出的类型是 std::pair<int, State>。
  std::cout << "count=" << status.first
            << ", state integer=" << static_cast<int>(status.second) << '\n';
  if (status.second == GEN_NEW_TRAJ)
    std::cout << "state name=GEN_NEW_TRAJ\n";
}
}  // namespace cpp_study

int main(int argc, char **argv)
{
  const std::string lesson = argc > 1 ? argv[1] : "1";
  if (lesson == "1") cpp_study::containers();
  else if (lesson == "2") cpp_study::references();
  else if (lesson == "3") cpp_study::classes();
  else if (lesson == "4") cpp_study::pointers();
  else if (lesson == "5") cpp_study::enumsAndPairs();
  else
  {
    std::cerr << "Choose one lesson: 1 containers, 2 references, 3 classes, 4 pointers, 5 enum/pair\n";
    return 1;
  }
  return 0;
}
