// C++14 面试综合教学程序：不加入 catkin 构建，不修改规划器或创建 ROS 节点。
// 编译/运行方式见 docs/面试/C++相关知识.md 的 C12 章节。
#include <Eigen/Core>
#include <atomic>
#include <condition_variable>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <queue>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

namespace interview {

void changeCopy(std::vector<int> values) { values.front() = 9; }
void changeReference(std::vector<int>& values) { values.front() = 9; }

bool values() {
  int value = 10;
  int* borrower = &value; // 借用自动对象，不能 delete。
  *borrower = 20;
  std::vector<int> points{1, 2};
  changeCopy(points);
  const int after_copy = points.front();
  changeReference(points);
  std::cout << "values: borrowed=" << value << " copy_first=" << after_copy
            << " ref_first=" << points.front() << '\n';
  return value == 20 && after_copy == 1 && points.front() == 9;
}

struct Task {
  virtual ~Task() = default;
  virtual int run() const = 0;
};

struct PlanningTask final : Task {
  int& destroyed;
  explicit PlanningTask(int& count) : destroyed(count) {}
  ~PlanningTask() override { ++destroyed; }
  int run() const override { return 42; }
};

bool polymorphism() {
  int destroyed = 0;
  int result = 0;
  {
    std::unique_ptr<Task> task = std::make_unique<PlanningTask>(destroyed);
    result = task->run();
  } // 基类虚析构使派生对象的析构路径正确执行。
  std::cout << "polymorphism: result=" << result << " destroyed=" << destroyed << '\n';
  return result == 42 && destroyed == 1;
}

struct Entry {
  double key;
  int id;
};
struct SmallerKeyFirst {
  bool operator()(const Entry& lhs, const Entry& rhs) const { return lhs.key > rhs.key; }
};

bool heap() {
  // 仅示意“新键重新入队 + 弹出时跳过旧条目”，不是完整 A*。
  std::priority_queue<Entry, std::vector<Entry>, SmallerKeyFirst> open;
  std::vector<double> best{5.0, 3.0};
  open.push({5.0, 0});
  open.push({3.0, 1});
  best[0] = 1.0;
  open.push({1.0, 0}); // 不在堆内部偷偷修改旧 Entry 的 key。
  std::vector<double> accepted;
  while (!open.empty()) {
    const Entry entry = open.top();
    open.pop();
    if (entry.key != best[entry.id]) continue; // 本例键为精确整数值，不讨论浮点误差策略。
    accepted.push_back(entry.key);
  }
  std::cout << "heap: accepted=" << accepted[0] << ',' << accepted[1] << '\n';
  return accepted == std::vector<double>{1.0, 3.0};
}

bool lambda() {
  int offset = 2;
  auto copy = [offset] { return offset; };
  auto reference = [&offset] { return offset; };
  offset = 5;
  auto owner = std::make_unique<int>(7);
  auto closure = [p = std::move(owner)] { return *p; };
  static_assert(!std::is_copy_constructible<decltype(closure)>::value,
                "unique ownership capture makes this closure move-only");
  // C++14 std::function 要求可复制目标，不能直接用它保存这个 move-only closure。
  std::function<int()> copyable = copy;
  std::cout << "lambda: copy=" << copyable() << " reference=" << reference()
            << " moved_resource=" << closure() << '\n';
  return copyable() == 2 && reference() == 5 && closure() == 7 && !owner;
}

struct JoinAll {
  std::vector<std::thread>& workers;
  ~JoinAll() {
    for (auto& worker : workers)
      if (worker.joinable()) worker.join();
  }
};

bool threading() {
  std::mutex mutex;
  int protected_count = 0;
  std::atomic<int> independent_count{0};
  {
    std::vector<std::thread> workers;
    JoinAll guard{workers}; // 线程创建过程中提前退出，也回收已经建立的线程。
    for (int i = 0; i < 4; ++i) {
      workers.emplace_back([&] {
        for (int k = 0; k < 1000; ++k) {
          {
            std::lock_guard<std::mutex> lock(mutex);
            ++protected_count;
          }
          // 这个原子量只计数，不发布其他共享数据；之后通过 join 等待完成。
          independent_count.fetch_add(1, std::memory_order_relaxed);
        }
      });
    }
  }

  std::condition_variable changed;
  bool ready = false;
  int observed = 0;
  {
    std::vector<std::thread> workers;
    JoinAll guard{workers};
    workers.emplace_back([&] {
      std::unique_lock<std::mutex> lock(mutex);
      changed.wait(lock, [&] { return ready; }); // 谓词处理提前通知和虚假唤醒。
      observed = 1;
    });
    {
      std::lock_guard<std::mutex> lock(mutex);
      ready = true;
    }
    changed.notify_one();
  }
  std::cout << "threading: mutex=" << protected_count
            << " atomic=" << independent_count.load() << " observed=" << observed << '\n';
  return protected_count == 4000 && independent_count.load() == 4000 && observed == 1;
}

struct LifetimeProbe {
  int& destroyed;
  explicit LifetimeProbe(int& count) : destroyed(count) {}
  ~LifetimeProbe() { ++destroyed; }
};

bool exception() {
  int destroyed = 0;
  try {
    auto owner = std::make_unique<LifetimeProbe>(destroyed);
    throw std::runtime_error("teaching example");
  } catch (const std::runtime_error&) {
    // owner 已随正常异常展开析构，而不是靠 catch 手写 delete。
  }
  std::cout << "exception: destroyed=" << destroyed << '\n';
  return destroyed == 1;
}

bool eigen() {
  Eigen::Vector3d p(1.0, 2.0, 3.0);
  Eigen::Vector3d q(2.0, 2.0, 2.0);
  auto lazy = p + q;          // 保存表达式，p/q 在本例中持续存活。
  auto materialized = (p + q).eval(); // 保存此刻计算结果。
  p.setZero();
  std::cout << "eigen: lazy_sum=" << lazy.sum()
            << " evaluated_sum=" << materialized.sum() << '\n';
  return lazy.sum() == 6.0 && materialized.sum() == 12.0;
}

} // namespace interview

int main(int argc, char** argv) {
  using Lesson = std::pair<std::string, bool (*)()>;
  const std::vector<Lesson> lessons{
      {"values", interview::values}, {"polymorphism", interview::polymorphism},
      {"heap", interview::heap}, {"lambda", interview::lambda},
      {"threading", interview::threading}, {"exception", interview::exception},
      {"eigen", interview::eigen}};
  const std::string requested = argc > 1 ? argv[1] : "all";
  bool found = false;
  for (const auto& lesson : lessons) {
    if (requested != "all" && requested != lesson.first) continue;
    found = true;
    if (!lesson.second()) return 1;
  }
  if (!found) {
    std::cerr << "Choose all, values, polymorphism, heap, lambda, threading, exception, or eigen\n";
    return 2;
  }
}
