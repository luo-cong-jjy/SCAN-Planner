// 独立 C++14 例子：作用域、成员生命周期、引用输出；不加入 catkin 构建。
#include <iostream>
#include <vector>

namespace {
void addCopy(std::vector<int> values) { values.push_back(9); }
void addReference(std::vector<int> &values) { values.push_back(9); }

struct Member {
  Member() { std::cout << "member constructed\n"; }
  ~Member() { std::cout << "member destroyed\n"; }
};

struct Planner {
  Member member; // 先构造成员，再执行 Planner 构造函数体。
  Planner() { std::cout << "planner constructor body\n"; }
  void init() { std::cout << "init called\n"; }
  ~Planner() { std::cout << "planner destructor body\n"; }
};
} // namespace

int main() {
  std::vector<int> points{1};
  addCopy(points);
  std::cout << "after copy: size=" << points.size() << '\n';
  addReference(points);
  std::cout << "after reference: size=" << points.size() << '\n';
  {
    Planner planner;
    planner.init();
    std::cout << "object still alive\n";
  } // 先执行析构函数体，再销毁成员。
  std::cout << "scope finished\n";
}
