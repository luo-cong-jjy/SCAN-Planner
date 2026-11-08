# 结合 SCAN-Planner 学习 C++

这份路线面向已经能看懂 if、for 等基本语法，但阅读 ROS/C++ 项目仍吃力的情况。目标是能独立追踪本项目的数据、对象和调用，再理解对应规划原理。

## 先把三层内容区分开

| 层次 | 本项目的例子 | 阅读时要确认什么 |
|---|---|---|
| C++ 语言 | 类、引用、指针、const、namespace、模板、auto、enum | 名字是什么类型，值怎样传递，对象什么时候存在 |
| 库接口 | std::vector、std::pair、智能指针、Eigen、ROS | 接口输入输出、是否修改对象、返回什么 |
| 项目与算法 | SCANReplanFSM、LocalTrajData、多项式、A*、B 样条 | 物理含义、调用情境、数学原理和失败处理 |

例如 heading 是程序员定义的变量名；Eigen::Vector3d 是库提供的类型；std::cos() 是标准库函数。一个名字属于哪层，要看声明和类型，不能只凭它附近有没有括号判断。

## 每块基础怎样算学会

每次选一块，安排约 45～60 分钟；这只是控制阅读范围，不是掌握期限。

1. 先读一个小例子，写下各变量的类型、含义和预期结果。
2. 合上解释，预测输出，再编译运行。
3. 只改一处：去掉引用、改变容器大小、更换状态或 reset 一个指针，解释结果为什么变化。
4. 回到指定项目片段，只解释本块相关的内容；外部函数先看用途和接口。
5. 隔一段时间再读同一片段，能不用照着注释解释输入、修改、输出和生命周期，再向下一块推进。

可以随时查速查表；反复遇到的点要配一次修改和运行。单纯增加注释，不能代替自己写一小段代码。

## 按顺序学习七块内容

### 1. 类型、变量、容器和基本表达式

先熟悉 double、bool、int、函数返回值、三目运算符，以及 vector、string 的常用操作。尖括号先理解成指定类型，例如 vector<double> 表示存放 double 的列表。

项目定位：[planner_manager.cpp](../src/planner/plan_manage/src/planner_manager.cpp) 的 point_set、start_end_derivatives、pseudo_arc_length；[scan_replan_fsm.cpp](../src/planner/plan_manage/src/scan_replan_fsm.cpp) 的关键点列表。

必须区分：

- reserve() 预留容量，元素数量不增加；resize() 改变元素数量。
- push_back() 追加，front()/back() 访问首尾，访问前要有元素。
- size() 是元素数量；capacity() 是容量，具体扩容数量不应凭想象认定。
- vector 的下标、Eigen 的分量访问和普通函数调用，语法相近但由接收对象类型决定含义。
- to_string(i) 将数值转为字符串；string::c_str() 提供只读字符指针给 C 风格接口。它们与字符串拼接一起用于参数键、日志和命令构造。

运行练习 1：先预测 reserve、追加、resize 三步的 size() 和首尾数值。再把 resize(5,0.0) 改为 resize(2)，预测哪些点被保留。

### 2. 值传递、引用、const 和作用域

项目定位：planner_manager.cpp 的 applyLinearZReference()，尤其是参数 points 的引用。

必须能解释：函数修改的是副本还是原数据；const 限制通过哪个名字修改；局部变量、类成员和 static 局部变量分别保存到什么时候。

同时区分声明与初始化：普通局部标量仅声明后可能没有可用初值，Eigen 的默认三维向量也不应假定为零。C++ 对象初始化与 nh.param() 的“参数读取失败后写入回退值”是两套不同机制。

运行练习 2：对比 changeCopy() 与 changeOriginal()。再将自己的练习函数参数改成 const 引用，尝试修改元素，读一次编译器报错后恢复。

符号速查：

| 写法 | 此处的意义 |
|---|---|
| double &height = value | 声明引用，height 是现有变量的别名 |
| double *pointer = &value | 声明指针，右侧 & 取得地址 |
| *pointer = 1.0 | 通过指针修改对象 |
| &SCANReplanFSM::execFSMCallback | 取得成员函数指针；具体回调绑定见第 7 块 |
| const vector<double> &values | 按引用接收列表，通过该参数只读访问 |

### 3. 类、对象、构造函数和头文件/实现文件

项目定位：[planner_manager.h](../src/planner/plan_manage/include/plan_manage/planner_manager.h) 与对应 cpp；[plan_container.hpp](../src/planner/plan_manage/include/plan_manage/plan_container.hpp) 的数据结构。

学习重点：成员变量、成员函数、public/private、构造/析构、类内声明与类外定义、namespace、作用域解析符 ::。

对照例子：

    Eigen::Vector3d heading(1.0, 0.0, 0.0);

这里类型是 Eigen::Vector3d，对象名是 heading，数值参数用于构造对象。

    bool SCANPlannerManager::reboundReplan(...)

这里是类成员函数的定义，前面的 bool 是返回类型。struct 和 class 都能包含数据及函数，默认访问权限不同；不是所有头文件都仅有声明，inline 等定义也会放在头文件中。

运行练习 3：Counter::increment() 是类外定义，counter 是对象。修改构造初值，解释两次调用的结果及 value_ 为什么能保留。

### 4. 原始指针与智能指针

项目定位：planner_manager.cpp 的 reset(new GridMap)、setEnvironment()，以及执行回调中的 LocalTrajData *info。

必须能回答：指向哪一个对象，是否复制对象，谁负责释放，何时可能为空。

- info 是访问现有 local_data_ 的普通指针，不负责创建或销毁轨迹。
- SCANPlannerManager::Ptr 与 BsplineOptimizer::Ptr 在当前项目中是独占智能指针类型。
- GridMap::Ptr、PlanningVisualization::Ptr 在当前项目中是共享智能指针类型。
- Ptr 只是项目里的类型别名，不能看见 ::Ptr 就一律认定为共享指针。
- ROS1 消息的 ConstPtr 还涉及生成消息中的指针别名，应按实际定义确认。

运行练习 4：比较独占指针 reset() 与共享指针 reset()；预测共享对象为什么还可以通过另一个持有者访问。直接看这个使用例子，再按需要查标准库接口，先掌握所有权规则。

进阶小改动：在自己的临时版本里给 Counter 添加析构打印，观察独占对象和共享对象分别在什么时候被销毁。

### 5. 模板常见读法、auto、enum 和 pair

项目定位：FSM 中的 FSM_EXEC_STATE、timesOfConsecutiveStateCalls()，以及各处 vector<Eigen::Vector3d>、head<2>()。

先掌握使用方式：

- 类型模板实参：pair<int, State>、vector<double>。
- 数值模板实参：Eigen 的 head<2>() 中，2 表示取两个分量。
- auto 在编译期推导类型；const auto & 常用于只读引用访问容器元素。
- enum 的状态名与用于打印的字符串是不同对象；按需要可转换成整数编号。
- pair 把两项数据装在同一对象中，通过 first、second 访问。
- 认识简单 lambda：例如多项式实现中的 `[](int x){...}`，以及 A* 中捕获外部变量的 `[&]`；先解释参数、返回值和捕获对象。

运行练习 5：改变 readStatus() 返回的次数和枚举值，预测结果。第一轮先学会阅读和使用已有模板；复杂模板推导、SFINAE、类型萃取可在确有需要时再展开。

### 6. Eigen：每次先说清尺寸和单位

项目定位：[uniform_bspline.cpp](../src/planner/bspline_opt/src/uniform_bspline.cpp)、planner_manager.cpp，配合 [论文公式对照](paper_formula_code_map.md)。

| 写法 | 首先要理解的用途 |
|---|---|
| Vector3d、VectorXd、MatrixXd | 固定三维向量、动态向量、动态矩阵 |
| point(2)、matrix.col(i) | 分量访问、列访问 |
| head<2>()、transpose() | 访问前两个分量、转置表达式 |
| norm()、squaredNorm() | 欧氏长度、长度平方 |
| dot()、cross() | 点积投影、三维叉积方向 |
| normalized()、setZero() | 求单位方向、原地置零 |
| solve() | 按矩阵和边界数据解方程 |

每个表达式标出“输入几行几列、输出几行几列、物理单位”。例如两个位置点相减是位移，norm() 后是距离；两个单位方向的点积则不是以米计的距离。

这里还会遇到运算符重载：向量的 (2) 可以访问分量，矩阵的 * 可以做乘法，Eigen 的 << 可以填入系数；cout 的 << 则负责输出。相同符号的具体操作由类型与上下文决定，先理解库的用法。

纸笔练习：用 (0,0,0) 和 (3,4,0) 算差向量、norm、squaredNorm、XY 分量；再用三行四列的控制点矩阵说明每一列是什么。先读一个公式对应的调用，必要时再展开 Eigen 的表达式实现。

### 7. ROS 回调、数据流和编译链接

项目定位：scan_planner_node.cpp → SCANReplanFSM::init() → createTimer()/subscribe()；再对照 CMakeLists.txt。

重点解释：

- main() 创建对象、注册接口，spin() 等待并调度回调。
- &类名::成员函数 指定哪个处理函数，this 指定在哪个现有对象上调用。
- 注册回调与立刻执行函数不同；成员状态可以在后续回调中继续使用。
- 发布器句柄、话题名称和消息类型分别是什么。
- 头文件提供声明/类型/部分实现；cpp 编译成目标文件，库实现通过链接组成可执行程序；launch 启动程序。
- 认识 #include 和日志宏的基本用途：ROS_WARN_STREAM 等宏负责构造日志，退出/重试等行为要继续追后面的控制语句。
- 区分 #ifdef 的编译期选择和普通 if 的运行时判断，例如可行性代价中的条件编译分支；定义了哪个宏会影响实际编入的公式。

练习：不运行机器人，画出“目标消息由谁接收、哪次定时回调调用管理器、轨迹发给谁、位姿反馈到哪”的四步流程，并给每一步标上函数名。

## 独立练习怎样运行

配套程序：[cpp_study_basics.cpp](../tools/cpp_study_basics.cpp)。它不加入规划器的 CMake 构建，只有标准库依赖。

在 SCAN-Planner 仓库根目录执行：

    g++ -std=c++14 -Wall -Wextra -pedantic tools/cpp_study_basics.cpp -o /tmp/scan_cpp_study
    /tmp/scan_cpp_study 1

每次只运行一项，将最后的数字改成 1～5。请先预测，再运行，最后复制到自己的临时文件中修改一处验证理解。

## 回到算法时的阅读方法

沿已有启动树和管理器调用顺序，每次取一个有明确输入输出的块：

1. 读签名和实际调用，写出输入/输出及单位。
2. 列出这一块真正修改的变量，区分共享数据和临时副本。
3. 写一句计算目的，再看公式和实现。
4. 用少量数字算一个例子，或实际运行观察对应话题/日志。
5. 碰到陌生接口，先查“它接收什么、返回什么、修改什么”，再判断是否需要进入它的定义。

例如理解 applyLinearZReference()，应能独立说明：引用传入候选点、计算水平累计距离、覆盖 Z、首尾高度来自局部起终点。能说明这些以后，再追论文的高度设计动机，比同时展开所有库实现更容易保持问题范围。

记录学习进度用“已经能自己写/改/解释什么”，而不是“读了多少行”。接下来先完成练习 1 和 2，再回到候选点重采样与高度处理；掌握数据怎样变化后，再进入 B 样条系数和优化梯度。
