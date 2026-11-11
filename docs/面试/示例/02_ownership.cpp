// 独立 C++14 例子：unique_ptr 移动、shared_ptr 共享、weak_ptr 观察。
#include <iostream>
#include <memory>
#include <string>
#include <utility>

struct Resource {
  std::string name;
  explicit Resource(std::string value) : name(std::move(value)) {
    std::cout << "construct " << name << '\n';
  }
  ~Resource() { std::cout << "destroy " << name << '\n'; }
};

int main() {
  std::cout << std::boolalpha;
  auto manager = std::make_unique<Resource>("manager");
  auto next_owner = std::move(manager);
  std::cout << "moved: source_empty=" << !manager
            << ", destination_valid=" << static_cast<bool>(next_owner) << '\n';
  next_owner.reset();

  auto display = std::make_shared<Resource>("display");
  auto other_owner = display;
  std::weak_ptr<Resource> observer = display;
  std::cout << "shared owners=" << display.use_count() << '\n';
  display.reset(); // other_owner 仍持有显示对象。
  {
    auto locked = observer.lock(); // 临时增加一个强所有者，离开作用域后释放。
    std::cout << "after first reset: alive=" << static_cast<bool>(locked)
              << ", name=" << locked->name << '\n';
  }
  other_owner.reset(); // 最后一个强所有者离开，对象在此销毁。
  std::cout << "after last reset: expired=" << observer.expired()
            << ", lock_failed=" << !observer.lock() << '\n';
}
