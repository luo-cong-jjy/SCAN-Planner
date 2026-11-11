// 独立 C++14 回调示意；不模拟 ROS 队列、线程或定时器实现。
#include <functional>
#include <iostream>
#include <memory>

struct Counter {
  int value = 0;
  void tick(int delta) { value += delta; }
};

int main() {
  Counter counter;
  void (Counter::*method)(int) = &Counter::tick;
  (counter.*method)(2);
  Counter *object = &counter; // 借用地址，不管理 counter 的生命周期。
  (object->*method)(3);
  std::cout << "member pointer value=" << counter.value << '\n';

  std::function<void()> callback = [object, method] {
    (object->*method)(4);
  };
  std::cout << "after registration=" << counter.value << '\n';
  callback(); // 手动调度，直到这里才执行保存的调用。
  std::cout << "after dispatch=" << counter.value << '\n';

  auto owner = std::make_shared<Counter>();
  std::weak_ptr<Counter> weak = owner;
  auto guarded_callback = [weak] {
    if (auto alive = weak.lock()) {
      alive->tick(1);
      std::cout << "guarded value=" << alive->value << '\n';
    } else {
      std::cout << "object expired; skip\n";
    }
  };
  guarded_callback();
  owner.reset();
  guarded_callback(); // 安全跳过，不访问已销毁对象。
}
