// 独立 C++14 例子：表达式产生闭包，调用对象时才执行函数体。
#include <iostream>

auto makeAdder(int offset) {
  return [offset](int x) { return x + offset; }; // 按值捕获，返回后副本仍有效。
}

int main() {
  const static auto factorial = [](int x) {
    int result = 1;
    for (int i = 2; i <= x; ++i) result *= i;
    return result;
  };
  std::cout << "factorial(0)=" << factorial(0)
            << ", factorial(4)=" << factorial(4) << '\n';

  int offset = 2;
  auto by_value = [offset](int x) { return x + offset; };
  auto by_reference = [&offset](int x) { return x + offset; };
  offset = 5;
  std::cout << "value=" << by_value(3)
            << ", reference=" << by_reference(3) << '\n';

  auto mutable_copy = [offset]() mutable { return ++offset; };
  std::cout << "mutable copy=" << mutable_copy() << ", outer=" << offset << '\n';
  auto returned_adder = makeAdder(7);
  std::cout << "returned lambda=" << returned_adder(3) << '\n';
}
