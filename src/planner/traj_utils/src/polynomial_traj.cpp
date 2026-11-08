/// @file
/// 用于全局参考和局部初始化的分段多项式生成器，不单独启动 ROS 节点。
/// 输入为空间点、每段时长及边界速度/加速度；按 XYZ 分别解线性系统，系数供 PolynomialTraj 求值。
/// 时间须为正，段数应与点数对应；轨迹生成本身不查询占用地图。
/// 阅读时先分清：Pos 是路径点，P* 是待求系数，D* 是端点状态；本文件的 Q 是代价矩阵，不是 B 样条控制点 Q_i。
#include <iostream>
#include <traj_utils/polynomial_traj.h>

// 接口来源速查：Eigen 的 MatrixXd/VectorXd 提供 Zero、size/rows/cols、block/segment/row、transpose/inverse 和 QR 求解等接口。
// C++ 提供“对象.成员()”和“类型::成员()”语法，具体计算由上述库实现；Eigen 头文件由 polynomial_traj.h 引入。
// pow 来自标准数学库，reverse、vector::begin/end 来自标准库；addSegment 是本项目 PolynomialTraj 类的接口。
// 相同的调用写法不代表相同来源，先看左侧对象/类型：例如 Time 为 Eigen 向量，它的 size() 就是 Eigen 成员函数。

/// @brief 构造分段五次多项式的边界映射和导数代价矩阵，分离固定/自由导数后求内部最优变量。
/// 保留原函数名 minSnapTraj，但下方 Q 的实际代价是 jerk（三阶导数）平方积分，不是四阶 snap 平方积分。
// 设 m 为段数：Pos 是 3×(m+1) 世界系路径点矩阵，每列 XYZ 的单位为米；Time 是 m 维各段时长（秒），不是累计到达时刻。
// start/end_vel 为整条轨迹首尾速度（m/s），start/end_acc 为首尾加速度（m/s²）；内部点的速度/加速度由本函数优化。
// 调用方在至少有 3 个路径点（m>=2）时使用此分段接口；每段时长应为正，点数和段数应匹配，本函数未统一检查。
// 每段局部时间从 0 开始，内部按升幂 x_k(t)=c0+c1*t+...+c5*t^5 求系数，最后再转换为容器要求的降幂顺序。
PolynomialTraj PolynomialTraj::minSnapTraj(const Eigen::MatrixXd &Pos, const Eigen::Vector3d &start_vel,
                                           const Eigen::Vector3d &end_vel, const Eigen::Vector3d &start_acc,
                                           const Eigen::Vector3d &end_acc, const Eigen::VectorXd &Time)
{
  // m=seg_num 是多项式段数：路径点 P0→P1→...→Pm 形成 m 段，每段 XYZ 各有 6 个待求系数。
  // Time 是 Eigen::VectorXd 动态向量，Time(k) 存第 k 段时长；size() 返回元素个数，所以时长有几项就有几段。
  int seg_num = Time.size();
  // 两段例子：P0→P1→P2，Time=[T0,T1]；中间点 P1 的位置已知，经过它的速度/加速度待求。
  // “每轴”指 X/Y/Z 中的一个坐标方向：三维轨迹分别用 x(t)、y(t)、z(t) 三个标量多项式表示。
  // 每段每轴是五次多项式 c0+c1*t+...+c5*t^5，含常数项共 6 个系数；两段的 X 共 2*6=12 项，Y/Z 各自也有 12 项。
  // 此时每轴有 12 个系数，固定边界量 8 项，自由导数 2 项；Ct 为 12×10，R 为 10×10。
  // poly_coeff 换成按“段”整理：每行一段，含 X/Y/Z 三组系数，共 3*6=18 列；两段时为 2×18，总共 36 项。
  // 一行的布局为 [X 的6项 | Y 的6项 | Z 的6项]，此时仍按升幂排列；这 18 项不是 18 个路径点。
  // [Eigen 构造] MatrixXd(rows,cols) 创建指定行列数的 double 矩阵；这里未置零，各项系数由下方求解后写入。
  Eigen::MatrixXd poly_coeff(seg_num, 3 * 6);
  // 分别保存全程 X/Y/Z 的系数向量，各有 6m 项；每连续 6 项对应一个路段，不是位置采样列表。
  // [Eigen 构造] VectorXd(n) 创建 n 个元素的动态向量；n 是长度，不是把每个元素都初始化为 n。
  Eigen::VectorXd Px(6 * seg_num), Py(6 * seg_num), Pz(6 * seg_num);

  // num_f 是固定边界量数，num_p 是自由边界量数；这里的 p 不是多项式次数，也不是空间位置。
  // “边界量”指每段首尾的位置、速度、加速度；“固定”指输入已指定、求解时保持，“自由”指由平滑性优化选择数值。
  // 按单个坐标轴计数：两段 P0→P1→P2 的固定量为 [x0,v0,a0,x1,x1,x2,v2,a2] 共 8 项，自由量为 [v1,a1] 共 2 项。
  // x1 按第一段终点、第二段起点各存一项，两项值相同；v1/a1 则由相邻两段共享，不代表允许连接点位置随意移动。
  int num_f, num_p; // number of fixed and free variables
  // 每段首尾各有位置/速度/加速度，共 6 项；num_d 统计各段端点状态展开后的总项数。
  int num_d;        // number of all segments' derivatives

  // 阶乘辅助函数 n!，用于 t^j 求 i 阶导数时的系数 j!/(j-i)!；参数 x 是整数阶次，不是空间 X 坐标。
  // 整个 [](int x){...} 是 lambda 表达式：产生一个支持 () 调用的对象，其类型（闭包类型）由编译器生成。
  // Factorial 是该对象的变量名，auto 从右侧推导其类型；调用写法像函数，但它不是普通函数声明中的函数名。
  // [] 是 lambda 必需的捕获列表，此处为空表示不捕获外部局部变量；x 是调用时传入的参数，写在 (int x) 中。
  // “外层”在这里指包住该 lambda 的 minSnapTraj()，例如它的局部变量 seg_num；阶乘计算只用自己的 x/fac/i，无需捕获这些外层量。
  // = 用右侧产生的对象初始化 Factorial；执行这条声明并不计算阶乘，后面 Factorial(i) 才执行 {} 中的代码。
  // const 禁止重新赋值该对象，不妨碍调用；static 使该对象在第一次执行到此声明时初始化，并跨调用保留。
  // 形如 [](int x)->int {...} 可显式写返回类型；本处由 return fac 推导为 int。变量初始化完成后须以 }; 结束声明。
  const static auto Factorial = [](int x) {
    int fac = 1;
    for (int i = x; i > 0; i--)
      fac = fac * i;
    return fac;
  };

  /* ---------- end point derivative ---------- */
  // Dx/Dy/Dz 是各轴“逐段端点状态”向量，每段的 6 项顺序为 [起点位置,终点位置,起点速度,终点速度,起点加速度,终点加速度]。
  // 初始只填已知位置和全程首尾导数；中间点的速度/加速度暂为零，后面会作为自由变量重新求解。
  // [Eigen 静态接口] VectorXd::Zero(n) 生成 n 项全为 0 的向量；不同于仅指定长度的 VectorXd(n)。
  Eigen::VectorXd Dx = Eigen::VectorXd::Zero(seg_num * 6);
  Eigen::VectorXd Dy = Eigen::VectorXd::Zero(seg_num * 6);
  Eigen::VectorXd Dz = Eigen::VectorXd::Zero(seg_num * 6);

  for (int k = 0; k < seg_num; k++)
  {
    /* position to derivative */
    // k 是段索引；第 k 段从 Pos 的第 k 列走到第 k+1 列，k*6 是它在端点向量中的起始偏移。
    Dx(k * 6) = Pos(0, k);
    Dx(k * 6 + 1) = Pos(0, k + 1);
    Dy(k * 6) = Pos(1, k);
    Dy(k * 6 + 1) = Pos(1, k + 1);
    Dz(k * 6) = Pos(2, k);
    Dz(k * 6 + 1) = Pos(2, k + 1);

    if (k == 0)
    {
      // 第一段的起点速度/加速度由调用方指定。
      Dx(k * 6 + 2) = start_vel(0);
      Dy(k * 6 + 2) = start_vel(1);
      Dz(k * 6 + 2) = start_vel(2);

      Dx(k * 6 + 4) = start_acc(0);
      Dy(k * 6 + 4) = start_acc(1);
      Dz(k * 6 + 4) = start_acc(2);
    }
    else if (k == seg_num - 1)
    {
      // 最后一段的终点速度/加速度由调用方指定；其余连接点导数留给后面的优化。
      Dx(k * 6 + 3) = end_vel(0);
      Dy(k * 6 + 3) = end_vel(1);
      Dz(k * 6 + 3) = end_vel(2);

      Dx(k * 6 + 5) = end_acc(0);
      Dy(k * 6 + 5) = end_acc(1);
      Dz(k * 6 + 5) = end_acc(2);
    }
  }

  /* ---------- Mapping Matrix A ---------- */
  // Ab 为单段 6×6 映射块，A 为全程 6m×6m 分块对角矩阵；数学关系为 A*系数向量=完整逐段端点状态。
  // 初始 Dx 中的内部导数零值只作占位，不是最终要求内部导数都为零；Y/Z 同理。
  // 因此 A 把“升幂系数”转换为上面的六项端点状态，A.inverse() 做反向转换。
  Eigen::MatrixXd Ab;
  // [Eigen 静态接口] MatrixXd::Zero(r,c) 生成 r×c 全零矩阵，随后只填入需要的对角块。
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(seg_num * 6, seg_num * 6);

  for (int k = 0; k < seg_num; k++)
  {
    Ab = Eigen::MatrixXd::Zero(6, 6);
    for (int i = 0; i < 3; i++)
    {
      // i=0/1/2 分别代表位置/速度/加速度；第 2i 行在段起点 t=0 求值，仅系数 c_i 乘 i! 后保留。
      Ab(2 * i, i) = Factorial(i);
      // 第 2i+1 行在段终点 t=Time(k) 求 i 阶导数：各系数乘 j!/(j-i)! * T^(j-i)。
      // [标准数学库] pow(base,exponent) 计算乘方，例如 pow(2.0,3)=8；Time(k) 是 Eigen 的分量访问，不是在调用名为 Time 的函数。
      for (int j = i; j < 6; j++)
        Ab(2 * i + 1, j) = Factorial(j) / Factorial(j - i) * pow(Time(k), j - i);
    }
    // 各段独立的系数到端点映射放在 A 的对角块，段与段之间的连接关系稍后由 Ct 表达。
    // [Eigen 视图] block(起始行,起始列,行数,列数) 访问子矩阵，下标从 0 开始；左侧赋值会直接写入 A 对应区域。
    A.block(k * 6, k * 6, 6, 6) = Ab;
  }

  /* ---------- Produce Selection Matrix C' ---------- */
  // 先将独立边界量排成 d_hat=[d_f;d_p]：前部已知/固定，后部是内部连接点的未知速度、加速度。
  // Ct 的大小为 6m×(4m+2)，把完整独立边界量 d_hat 展开到各段端点；C 是 Ct 的转置，不是其逆矩阵。
  // 相邻段在同一连接点共用一个速度、一个加速度，因此 Ct 会把同一自由变量写到相邻两段，保证这两阶导数连续。
  Eigen::MatrixXd Ct, C;

  // 固定量：各段首尾位置共 2m 项（连接位置按相邻两段各存一次），加上全程首尾速度/加速度共 4 项。
  num_f = 2 * seg_num + 4; // 3 + 3 + (seg_num - 1) * 2 = 2m + 4
  // 自由量：m-1 个内部连接点，每个有速度、加速度两个未知量，共 2m-2 项（每个坐标轴分别求）。
  num_p = 2 * seg_num - 2; //(seg_num - 1) * 2 = 2m - 2
  num_d = 6 * seg_num;
  Ct = Eigen::MatrixXd::Zero(num_d, num_f + num_p);
  // 以下 0/1 元素决定“逐段端点的某一行，取 d_hat 中的哪一项”；先设置第一段和最后一段，再填中间段。
  // 固定部分前 3 项为全程起点 p/v/a，后面接各段位置，末尾放全程终点 v/a；自由部分按内部点的 v/a 成对排列。
  Ct(0, 0) = 1;
  Ct(2, 1) = 1;
  Ct(4, 2) = 1; // stack the start point
  Ct(1, 3) = 1;
  Ct(3, 2 * seg_num + 4) = 1;
  Ct(5, 2 * seg_num + 5) = 1;

  Ct(6 * (seg_num - 1) + 0, 2 * seg_num + 0) = 1;
  Ct(6 * (seg_num - 1) + 1, 2 * seg_num + 1) = 1; // Stack the end point
  Ct(6 * (seg_num - 1) + 2, 4 * seg_num + 0) = 1;
  Ct(6 * (seg_num - 1) + 3, 2 * seg_num + 2) = 1; // Stack the end point
  Ct(6 * (seg_num - 1) + 4, 4 * seg_num + 1) = 1;
  Ct(6 * (seg_num - 1) + 5, 2 * seg_num + 3) = 1; // Stack the end point

  for (int j = 2; j < seg_num; j++)
  {
    // j-1 为中间段索引；它的首尾 v/a 引用相邻连接点的同一对自由量，不为两段分别创建独立导数。
    Ct(6 * (j - 1) + 0, 2 + 2 * (j - 1) + 0) = 1;
    Ct(6 * (j - 1) + 1, 2 + 2 * (j - 1) + 1) = 1;
    Ct(6 * (j - 1) + 2, 2 * seg_num + 4 + 2 * (j - 2) + 0) = 1;
    Ct(6 * (j - 1) + 3, 2 * seg_num + 4 + 2 * (j - 1) + 0) = 1;
    Ct(6 * (j - 1) + 4, 2 * seg_num + 4 + 2 * (j - 2) + 1) = 1;
    Ct(6 * (j - 1) + 5, 2 * seg_num + 4 + 2 * (j - 1) + 1) = 1;
  }

  // [Eigen 接口] transpose() 返回转置表达式，行列互换，不原地改变 Ct；赋给 C 后，C 保存转置后的矩阵。
  C = Ct.transpose();

  // 将初始端点状态组织到固定/自由排列。当前内部导数初始为零，后面的 Dxf 等只取固定部分，自由部分另行求解。
  Eigen::VectorXd Dx1 = C * Dx;
  Eigen::VectorXd Dy1 = C * Dy;
  Eigen::VectorXd Dz1 = C * Dz;

  /* ---------- minimum snap matrix ---------- */
  // Q 为系数空间的平滑代价矩阵（6m×6m），使 Px^T*Q*Px = 各段积分 ∫[x'''(t)]²dt，Y/Z 同理。
  // 名称沿用原实现；i/j 从 3 开始且各乘三项下降因子，实际对应三阶导数 jerk，而非四阶 snap。
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(seg_num * 6, seg_num * 6);

  for (int k = 0; k < seg_num; k++)
  {
    for (int i = 3; i < 6; i++)
    {
      for (int j = 3; j < 6; j++)
      {
        // 两个三阶导数基函数相乘后为 t^(i+j-6)；积分到 T 得 T^(i+j-5)/(i+j-5)，即下面的时间因子。
        Q(k * 6 + i, k * 6 + j) =
            i * (i - 1) * (i - 2) * j * (j - 1) * (j - 2) / (i + j - 5) * pow(Time(k), (i + j - 5));
      }
    }
  }

  /* ---------- R matrix ---------- */
  // 代入 Px=A^-1*Ct*Dx1，将系数空间代价转到边界量空间：Jx=Dx1^T*R*Dx1；R 大小为 (4m+2)×(4m+2)。
  // R 的分块随后区分固定量和自由量，使优化只求内部连接点的速度/加速度。
  // [Eigen 链式调用] A.transpose().inverse() 按数学含义为先取 A^T，再对 A^T 求逆，即 A^(-T)；不是一个名字很长的新函数。
  // inverse() 求矩阵逆：可逆方阵 M 的逆满足 M*M^-1=I（I 为单位矩阵），不是逐个元素取倒数，也不修改原矩阵。
  // 此链式表达式要求 A 可逆；本例矩阵来自给定段时长的边界映射，时间等输入须满足生成器的有效条件。
  Eigen::MatrixXd R = C * A.transpose().inverse() * Q * A.inverse() * Ct;

  // Dxf/Dyf/Dzf：固定边界量 d_f（各 2m+4 项），由输入路径位置及整条轨迹首尾导数给定，优化时不改变。
  Eigen::VectorXd Dxf(2 * seg_num + 4), Dyf(2 * seg_num + 4), Dzf(2 * seg_num + 4);

  // [Eigen 视图] segment(起始下标,元素数) 访问向量的连续一段；这里从 0 开始取固定量部分，再复制到 Dxf。
  Dxf = Dx1.segment(0, 2 * seg_num + 4);
  Dyf = Dy1.segment(0, 2 * seg_num + 4);
  Dzf = Dz1.segment(0, 2 * seg_num + 4);

  // R=[Rff Rfp; Rpf Rpp]；下标 f/p 分别表示固定/自由部分，如 Rfp 为固定量与自由量之间的耦合块。
  Eigen::MatrixXd Rff(2 * seg_num + 4, 2 * seg_num + 4);
  Eigen::MatrixXd Rfp(2 * seg_num + 4, 2 * seg_num - 2);
  Eigen::MatrixXd Rpf(2 * seg_num - 2, 2 * seg_num + 4);
  Eigen::MatrixXd Rpp(2 * seg_num - 2, 2 * seg_num - 2);

  Rff = R.block(0, 0, 2 * seg_num + 4, 2 * seg_num + 4);
  Rfp = R.block(0, 2 * seg_num + 4, 2 * seg_num + 4, 2 * seg_num - 2);
  Rpf = R.block(2 * seg_num + 4, 0, 2 * seg_num - 2, 2 * seg_num + 4);
  Rpp = R.block(2 * seg_num + 4, 2 * seg_num + 4, 2 * seg_num - 2, 2 * seg_num - 2);

  /* ---------- close form solution ---------- */

  // Dxp/Dyp/Dzp：内部连接点的自由导数 d_p（各 2m-2 项），不是内部点的位置；内部位置已被路径点固定。
  // 对 J=[d_f;d_p]^T*R*[d_f;d_p] 关于 d_p 求导并令其为零：d_p=-Rpp^-1*Rpf*d_f。
  // 理论上 R 对称，所以代码使用 Rfp.transpose() 代替 Rpf；Rff/Rpf 在此保留完整分块表达。
  Eigen::VectorXd Dxp(2 * seg_num - 2), Dyp(2 * seg_num - 2), Dzp(2 * seg_num - 2);
  Dxp = -(Rpp.inverse() * Rfp.transpose()) * Dxf;
  Dyp = -(Rpp.inverse() * Rfp.transpose()) * Dyf;
  Dzp = -(Rpp.inverse() * Rfp.transpose()) * Dzf;

  // 将求得的内部速度/加速度写回独立边界向量的自由部分，至此首尾和内部导数均已确定。
  // segment() 放在赋值左侧时，写入的是原 Dx1 的对应区间，不会另建一份独立向量再丢弃。
  Dx1.segment(2 * seg_num + 4, 2 * seg_num - 2) = Dxp;
  Dy1.segment(2 * seg_num + 4, 2 * seg_num - 2) = Dyp;
  Dz1.segment(2 * seg_num + 4, 2 * seg_num - 2) = Dzp;

  // 先用 Ct 将共享导数展开到各段端点，再用 A^-1 求每段六个系数，输出仍按升幂排列。
  Px = (A.inverse() * Ct) * Dx1;
  Py = (A.inverse() * Ct) * Dy1;
  Pz = (A.inverse() * Ct) * Dz1;

  for (int i = 0; i < seg_num; i++)
  {
    // 将第 i 段的 X/Y/Z 系数从三个长向量分别取出，横向拼到 poly_coeff 的一行。
    // segment(...,6) 是 6×1 列向量，transpose() 转为 1×6 行，才能与左侧 1×6 子矩阵的尺寸匹配。
    poly_coeff.block(i, 0, 1, 6) = Px.segment(i * 6, 6).transpose();
    poly_coeff.block(i, 6, 1, 6) = Py.segment(i * 6, 6).transpose();
    poly_coeff.block(i, 12, 1, 6) = Pz.segment(i * 6, 6).transpose();
  }

  /* ---------- use polynomials ---------- */
  // 将求出的每段系数与对应时长封装成 PolynomialTraj，返回后可按时间查询位置及导数。
  PolynomialTraj poly_traj;
  // [Eigen 接口] rows()/cols() 分别返回行数/列数；此处每行对应一段，所以遍历 rows() 次。
  for (int i = 0; i < poly_coeff.rows(); ++i)
  {
    // cx/cy/cz 为当前段各坐标轴的六个系数，暂时复制上述升幂结果。
    vector<double> cx(6), cy(6), cz(6);
    for (int j = 0; j < 6; ++j)
    {
      cx[j] = poly_coeff(i, j), cy[j] = poly_coeff(i, j + 6), cz[j] = poly_coeff(i, j + 12);
    }
    // 轨迹容器按 [c5,c4,c3,c2,c1,c0] 降幂存放，因此需要反转；这只是系数顺序转换，不是倒放轨迹。
    // [标准库] reverse(begin,end) 原地反转范围；vector::begin() 指向首元素，end() 指向末元素之后，范围左闭右开。
    reverse(cx.begin(), cx.end());
    reverse(cy.begin(), cy.end());
    reverse(cz.begin(), cz.end());
    // ts 是第 i 段的总时长（s），不是管理器中同名的 B 样条采样/节点时间间隔。
    double ts = Time(i);
    // [项目接口] addSegment() 定义在 polynomial_traj.h，将当前段的三轴系数与时长追加到轨迹容器。
    poly_traj.addSegment(cx, cy, cz, ts);
  }

  return poly_traj;
}

/// @brief 根据一段的首尾位置、速度、加速度解六个系数，分别求 XYZ，再存入轨迹容器。
// 以 X 为例：x(t)=a0+a1*t+...+a5*t^5，给定正时长后，首尾各三个条件 x/v/a 构成六个线性方程。
// 下方 C 是系数到边界状态的映射，Bx/By/Bz 是要求的边界状态；solve() 求系数，不查询地图或搜索障碍间路线。
// t 为这一段的执行时长（s），不是当前采样时刻；给定正时长和首尾 p/v/a 后，每轴的六个系数由线性方程确定。
// 此函数直接按 [c5,c4,c3,c2,c1,c0] 降幂求解，与上面多段函数内部的升幂排列不同。
PolynomialTraj PolynomialTraj::one_segment_traj_gen(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                                    const Eigen::Vector3d &end_pt, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc,
                                                    double t)
{
  // 此处的 C 是单段 6×6“系数→边界状态”矩阵，含义不同于 minSnapTraj() 中的选择矩阵转置 C。
  // Crow 是 1×6 临时行，用来依次填入终点位置、一阶导数、二阶导数的系数权重。
  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(6, 6), Crow(1, 6);
  // Bx/By/Bz 是各轴要求的六项边界值，顺序为 [起点位置,起点速度,起点加速度,终点位置,终点速度,终点加速度]。
  Eigen::VectorXd Bx(6), By(6), Bz(6);

  // 在起点局部时间 0：位置=c0，速度=c1，加速度=2*c2；列号按降幂系数顺序，因此位于第 5/4/3 列。
  C(0, 5) = 1;
  C(1, 4) = 1;
  C(2, 3) = 2;
  // 在终点局部时间 t：用 [t^5,t^4,t^3,t²,t,1] 乘系数向量得到位置。
  Crow << pow(t, 5), pow(t, 4), pow(t, 3), pow(t, 2), t, 1;
  // [Eigen 视图] row(i) 访问第 i 行；C.row(3)=Crow 将 Crow 的六项直接写到 C 的第 4 行。
  C.row(3) = Crow;
  // 对同一多项式求一阶导数后，在终点 t 求值，得到终点速度约束行。
  Crow << 5 * pow(t, 4), 4 * pow(t, 3), 3 * pow(t, 2), 2 * t, 1, 0;
  C.row(4) = Crow;
  // 再求一次导数，得到终点加速度约束行；六行合起来表达该段完整的首尾状态要求。
  Crow << 20 * pow(t, 3), 12 * pow(t, 2), 6 * t, 2, 0, 0;
  C.row(5) = Crow;

  // 三个 B 向量数值分别来自 XYZ 分量，使用同一个 C 求解；每轴边界值包含 m、m/s、m/s² 三种单位。
  Bx << start_pt(0), start_vel(0), start_acc(0), end_pt(0), end_vel(0), end_acc(0);
  By << start_pt(1), start_vel(1), start_acc(1), end_pt(1), end_vel(1), end_acc(1);
  Bz << start_pt(2), start_vel(2), start_acc(2), end_pt(2), end_vel(2), end_acc(2);

  // Cofx/Cofy/Cofz 是解出的各轴降幂系数；给定有效时长时，以 QR 分解求 C*系数=B 的线性系统。
  // c_j 的单位随幂次为 m/s^j，不能将整个系数向量理解为六个位置点。
  // [Eigen 链式调用] colPivHouseholderQr() 创建列主元 Householder QR 分解求解器，solve(Bx) 用它求解 C*系数=Bx。
  // solve() 返回解向量而不修改 Bx；这与上面的显式 inverse() 求逆写法不同，目的都是完成所需线性代数计算。
  Eigen::VectorXd Cofx = C.colPivHouseholderQr().solve(Bx);
  Eigen::VectorXd Cofy = C.colPivHouseholderQr().solve(By);
  Eigen::VectorXd Cofz = C.colPivHouseholderQr().solve(Bz);

  // 将 Eigen 系数向量复制成容器 addSegment() 接收的标准 vector；此处已经降幂，不需要 reverse()。
  vector<double> cx(6), cy(6), cz(6);
  for (int i = 0; i < 6; i++)
  {
    cx[i] = Cofx(i);
    cy[i] = Cofy(i);
    cz[i] = Cofz(i);
  }

  // poly_traj 是要返回的轨迹对象；addSegment() 保存三轴系数与执行时长，供 evaluate()/evaluateVel()/evaluateAcc() 使用。
  // getTimeSum() 等累计统计依赖调用 init()；本处只追加一段，没有同步计算累计统计。
  PolynomialTraj poly_traj;
  poly_traj.addSegment(cx, cy, cz, t);

  return poly_traj;
}
