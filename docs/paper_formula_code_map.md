# SCAN-Planner：论文公式与源码逐项对照

本文对应仓库内的 [SCAN-Planner 论文](<../Zheng 等 - 2026 - SCAN-Planner Spatial Collision-Aware Local Planning for Route-Guided Long-Range Quadruped Navigatio.pdf>)，按 PDF 页面计页码。论文共有 15 个编号公式；下文逐项列出数学含义、当前工作区中的实际代码及实现差异。核对日期：2026-11-05。

代码中已增加“论文式 (编号)”注释，可直接搜索定位。行号会随后续注释变化，函数名和公式编号可作为持续使用的入口。

## 怎样从启动命令追到这些公式

    roslaunch scan_planner run.launch
      → advanced_param.xml 启动 scan_planner_node
      → scan_planner_node.cpp → SCANReplanFSM
      → SCANPlannerManager::reboundReplan()
          ├─ applyLinearZReference()：式 (7)、(8)
          ├─ UniformBspline：式 (1)
          ├─ BsplineOptimizer::initControlPoints()
          │   ├─ 朝向与双圆柱查询：式 (2)～(5)
          │   └─ 投影 A*：式 (14)、(15)
          └─ BsplineOptimizer 的代价与梯度：式 (6)、(9)～(13)

式号顺序是论文的叙述顺序，不表示程序执行顺序。地图在进入局部规划前已初始化，碰撞查询使用其维护的膨胀占用数据。

## 常用符号先与变量对齐

| 论文符号 | 代码中的表示 | 含义 |
|---|---|---|
| $Q_i$ | control_points_.col(i)、cps_.points.col(i)、q.col(i) | B 样条控制点，单位 m；不是轨迹必经采样点，也不是实际里程计位置 |
| $\Delta t$ | interval_、bspline_interval_、管理器局部变量 ts | 样条名义节点间隔，单位 s；区别于 ROS 回调周期和计算耗时 |
| $V_i,A_i,J_i$ | 导数控制点、vi、ai、jerk 相关差分 | 速度、加速度、jerk；个别代价计算省略了时间缩放，详见式 (12) |
| $\psi_i$ | estimateControlPointYaw() 返回值 | 控制点局部方向诱导的 yaw，单位 rad |
| $d_{\rm off}$ | mp_.double_cylinder_offset_ | 前后圆柱中心相对机身参考点的偏移长度，单位 m |
| $a_{ij}$ | cps_.base_point[i][j] | 障碍基点，单位 m |
| $v_{ij}$ | cps_.direction[i][j] | 单位推离方向，无单位；与速度控制点 $V_i$ 不同 |
| $s_f$ | cps_.clearance | 碰撞代价的期望余量，单位 m；不是双圆柱地图膨胀半径的另一个名字 |
| $\lambda_s,\lambda_c,\lambda_f$ | lambda1_、new_lambda2_、lambda3_ | Rebound 的平滑、碰撞、可行性权重 |

<a id="eq-1"></a>
## 式 (1)：速度、加速度和 jerk 控制点

**论文：第 3 页，IV-A。**

$$
V_i=\frac{Q_{i+1}-Q_i}{\Delta t},\qquad
A_i=\frac{V_{i+1}-V_i}{\Delta t},\qquad
J_i=\frac{A_{i+1}-A_i}{\Delta t}.
$$

来源：[uniform_bspline.cpp](../src/planner/bspline_opt/src/uniform_bspline.cpp)，getDerivativeControlPoints() 和 getDerivative()。

    ctp.col(i) =
        p_ * (control_points_.col(i + 1) - control_points_.col(i))
        / (u_(i + p_ + 1) - u_(i + 1));

代码采用一般 B 样条导数公式。均匀节点下，分母为 $p_b\Delta t$，与分子的次数因子抵消，得到论文中的相邻控制点差分除以 $\Delta t$。getDerivative() 降低次数并截去首尾节点，重复求导可得到速度、加速度和 jerk 样条。

优化器 calcFeasibilityCost() 中的 vi、ai 也直接使用均匀节点差分；calcSmoothnessCost() 的 jerk 则省略了时间缩放，见式 (12)。

<a id="eq-2"></a>
## 式 (2)：由局部切向诱导 yaw

**论文：第 3 页，IV-A。**

$$
\psi_i=\operatorname{atan2}
\big((Q_{i+1}-Q_{i-1})_y,\ (Q_{i+1}-Q_{i-1})_x\big).
$$

来源：[bspline_optimizer.cpp](../src/planner/bspline_opt/src/bspline_optimizer.cpp)，estimateControlPointYaw() → estimateSegmentYaw()。

    int prev_id = std::max(0, id - 1);
    int next_id = std::min(static_cast<int>(q.cols()) - 1, id + 1);
    return estimateSegmentYaw(q.col(prev_id), q.col(next_id));

下层对两点的 XY 差调用 atan2(dy, dx)。内部控制点与论文表达对应；边界截断索引。XY 差的平方长度不足 1e-8 时，优化器回退为 0 rad，属于额外的退化处理。

FSM 的 estimateYawFromSegment() 用相邻轨迹采样点而非前后控制点计算方向，退化时回退里程计朝向。两者用途相近，但不能当作完全相同的取样与回退方式。

<a id="eq-3"></a>
## 式 (3)：机身坐标系中的前后圆柱中心

**论文：第 3 页，IV-B。**

$$
s_{b,1}=(d_{\rm off},0,0)^\top,\qquad
s_{b,2}=(-d_{\rm off},0,0)^\top.
$$

来源：[grid_map.h](../src/planner/plan_env/include/plan_env/grid_map.h)，getInflateOccupancy(pos, yaw)。

代码没有单独声明两个 s_b 向量，而是将它们隐含在正、负 double_cylinder_offset_ 中，并直接计算式 (4) 的结果。因此该式对应的是模型定义，不是一个同名函数。

圆柱半径和上下膨胀距离在 [grid_map.cpp](../src/planner/plan_env/src/grid_map.cpp) 的 rebuildInflationOffsets() 中离散成膨胀体素偏移。论文讨论的可跨越台阶高度不是在此处自动估计的，当前实现读取相应配置。

<a id="eq-4"></a>
## 式 (4)：将圆柱中心变换到世界坐标系

**论文：第 4 页，IV-B。**

$$
s_{k,j}=R_z(\psi_k)s_{b,j}+Q_k,\qquad j\in\{1,2\}.
$$

来源：[grid_map.h](../src/planner/plan_env/include/plan_env/grid_map.h)，getInflateOccupancy(pos, yaw)。

    Eigen::Vector3d heading(std::cos(yaw), std::sin(yaw), 0.0);
    Eigen::Vector3d front = pos + mp_.double_cylinder_offset_ * heading;
    Eigen::Vector3d rear  = pos - mp_.double_cylinder_offset_ * heading;

pos 对应检查位置 $Q_k$。由于机身系中心只沿 X 轴偏移，矩阵乘法可化简为上面的正、负方向偏移；无需显式构造 Rz 矩阵。两个查询中心的 Z 均为 pos.z。

FSM 的 publishSelfInflationMarker() 是显示入口。真实碰撞查询应以上述地图接口为准，尤其是上下显示厚度不对称时，不能仅根据 Marker 判断实现等价性。

<a id="eq-5"></a>
## 式 (5)：两个中心的占用合成为机身占用

**论文：第 4 页，IV-B。**

$$
H_k(Q_k,\psi_k)=\max_{j\in\{1,2\}}\chi(s_{k,j}).
$$

来源：[grid_map.h](../src/planner/plan_env/include/plan_env/grid_map.h)，getInflateOccupancy() 和 getInflateOccupancyFromBuffer()。

    int front_occ = getInflateOccupancyFromBuffer(front, md_.occupancy_buffer_inflate_);
    if (front_occ != 0) return front_occ;
    return getInflateOccupancyFromBuffer(rear, md_.occupancy_buffer_inflate_);

对于地图内的二值 0/1，任一占用就返回 1，逻辑上等价于 OR/max；两个都为 0 才返回 0。

**实现差异：** 地图接口还会返回 -1 表示越界，不能把包含 -1 的调用直接当作论文的二值 max。例如前点越界时会直接返回 -1，不再检查后点。0 表示没有被该膨胀层标为占用，也不能单凭此值证明已经观测到真实自由空间。

<a id="eq-6"></a>
## 式 (6)：Rebound 的总目标函数

**论文：第 4 页，IV-C。**

$$
\min_Q J=\lambda_cJ_c+\lambda_sJ_s+\lambda_fJ_f.
$$

来源：[bspline_optimizer.cpp](../src/planner/bspline_opt/src/bspline_optimizer.cpp)，combineCostRebound()，以及调用 L-BFGS 的 rebound_optimize()。

    f_combine = lambda1_ * f_smoothness
              + new_lambda2_ * f_distance
              + lambda3_ * f_feasibility;

对应关系：lambda1_ 为平滑权重，new_lambda2_ 为本轮碰撞权重，lambda3_ 为可行性权重。碰撞权重可在重试中调整，不应始终等同于最初读入的 lambda2_。

求解变量来自内部控制点，首尾各 order_ 个控制点固定。combineCostRebound() 还将总梯度的 Z 行置零，实际采用水平更新。Refine 的 combineCostRefine() 使用参考贴合项，不能直接对应成同一组三项总代价。

<a id="eq-7"></a>
## 式 (7)：水平累计弧长

**论文：第 4 页，IV-C.1。**

$$
\ell_i=\sum_{m=1}^{i}\|r_{m,xy}-r_{m-1,xy}\|,\qquad L=\ell_{N_p}.
$$

来源：[planner_manager.cpp](../src/planner/plan_manage/src/planner_manager.cpp)，匿名命名空间中的 applyLinearZReference()。

    accumulated_xy_length[i] = accumulated_xy_length[i - 1]
        + (points[i].head<2>() - points[i - 1].head<2>()).norm();
    const double total_xy_length = accumulated_xy_length.back();

points 对应候选采样点 $r_i$，不是控制点 $Q_i$。accumulated_xy_length[i] 为 $\ell_i$，total_xy_length 为 $L$，单位均为 m；它们沿候选 XY 折线累计，不是首尾直线距离或实际机器人行驶里程。

<a id="eq-8"></a>
## 式 (8)：按水平进度分配高度

**论文：第 4 页，IV-C.1。**

$$
z_i=z_s+\alpha_i(z_g-z_s),\qquad
\alpha_i=
\begin{cases}
\ell_i/L,&L>0,\\
i/N_p,&L=0.
\end{cases}
$$

来源：[planner_manager.cpp](../src/planner/plan_manage/src/planner_manager.cpp)，applyLinearZReference()。

    const double ratio = total_xy_length > 1e-6
        ? accumulated_xy_length[i] / total_xy_length
        : static_cast<double>(i) / static_cast<double>(points.size() - 1);
    points[i](2) = start_z + ratio * (target_z - start_z);

ratio 对应无量纲 $\alpha_i$，start_z/target_z 对应局部起点、局部目标高度。保持 XY 不变，覆盖每个采样点的 Z。

代码将论文的零长度情形扩展为 $L\leq10^{-6}$ m，并在循环外处理空列表/单点列表、显式固定首尾高度。单点时只设置起点高度。后续 Z 梯度置零是另一项实施措施，不是这条插值公式本身。

<a id="eq-9"></a>
## 式 (9)：沿推离方向的障碍投影距离

**论文：第 4 页，IV-C.2。**

$$
d_{ij}=(Q_i-a_{ij})^\top v_{ij}.
$$

来源：[bspline_optimizer.cpp](../src/planner/bspline_opt/src/bspline_optimizer.cpp)，calcDistanceCostRebound()。

    double dist = (cps_.points.col(i) - cps_.base_point[i][j])
        .dot(cps_.direction[i][j]);

dist 对应 $d_{ij}$，base_point 对应基点 $a_{ij}$，direction 对应单位方向 $v_{ij}$。这些基点/方向由 initControlPoints() 利用 A* 引导构造，也可由 check_collision_and_rebound() 更新。

这是沿指定方向的有符号投影距离，不是全局最近障碍的欧氏距离或 ESDF 值。

<a id="eq-10"></a>
## 式 (10)：累加碰撞惩罚

**论文：第 4 页，IV-C.2。**

$$
J_c=\sum_{i,j}\rho(c_{ij}),\qquad c_{ij}=s_f-d_{ij}.
$$

来源：[bspline_optimizer.cpp](../src/planner/bspline_opt/src/bspline_optimizer.cpp)，calcDistanceCostRebound() 中 dist_err 与 cost 的分段累加。

    double dist_err = cps_.clearance - dist;

dist_err 对应 $c_{ij}$，cps_.clearance 对应 $s_f$。在正余量 $s_f>0$ 下，当前代码选用：

$$
\rho(c)=
\begin{cases}
0,&c<0,\\
c^3,&0\leq c<s_f,\\
3s_fc^2-3s_f^2c+s_f^3,&c\geq s_f.
\end{cases}
$$

论文以 $\rho$ 概括光滑单侧惩罚；代码给出其具体分段形式。安全余量已满足时该项为零，余量不足时才惩罚。二值地图用于检测碰撞、生成基点/方向，不直接对 0/1 占用值求导。

<a id="eq-11"></a>
## 式 (11)：碰撞项梯度

**论文：第 4 页，IV-C.2。**

$$
\frac{\partial J_c}{\partial Q_i}=-\sum_j\rho'(c_{ij})v_{ij}.
$$

来源：[bspline_optimizer.cpp](../src/planner/bspline_opt/src/bspline_optimizer.cpp)，calcDistanceCostRebound() 中的 gradient.col(i)。

    gradient.col(i) += -3.0 * dist_err * dist_err * dist_grad;
    gradient.col(i) += -(2.0 * a * dist_err + b) * dist_grad;

两句位于不同区间分支，分别对应 $\rho'(c)=3c^2$ 与 $\rho'(c)=6s_fc-3s_f^2$，不会对同一对基点同时执行。

单看碰撞项，梯度带负号；梯度下降减去梯度后，会倾向于沿正 direction 增大投影距离。最终更新还受到其他代价、固定边界控制点以及 Z 梯度抑制影响，gradient 不是直接加到控制点上的位移。

<a id="eq-12"></a>
## 式 (12)：jerk 平滑代价

**论文：第 4 页，IV-C.3。**

$$
J_s=\sum_k\|J_k\|^2.
$$

来源：[bspline_optimizer.cpp](../src/planner/bspline_opt/src/bspline_optimizer.cpp)，calcSmoothnessCost() 默认的 jerk 分支。

    jerk = q.col(i + 3) - 3 * q.col(i + 2)
         + 3 * q.col(i + 1) - q.col(i);
    cost += jerk.squaredNorm();

三阶差分抑制控制点的剧烈弯折变化，使对应轨迹的加速度变化更平缓。

**时间尺度差异：** 按式 (1)，物理 jerk 控制点还要除以 $\Delta t^3$。因此固定均匀 $\Delta t$ 时，这里的代价值为论文物理 jerk 平方和的 $\Delta t^6$ 倍。固定时间间隔下该比例可吸收到权重，但当前语句没有显式做此换算；不能把打印的 cost 直接当成物理 jerk 平方和，也不能跨不同 ts 忽略这项尺度差别。

函数还提供加速度平滑备用分支；论文式 (12) 对应默认的 jerk 分支。

<a id="eq-13"></a>
## 式 (13)：速度与加速度可行性惩罚

**论文：第 4 页，IV-C.3。**

$$
J_f=\sum_kF(\|V_k\|,v_m)+\sum_kF(\|A_k\|,a_m),\qquad
F(c,c_m)=\max(c-c_m,0)^2.
$$

来源：[bspline_optimizer.cpp](../src/planner/bspline_opt/src/bspline_optimizer.cpp)，calcFeasibilityCost()。

    Eigen::Vector3d vi = (q.col(i + 1) - q.col(i)) / ts;
    Eigen::Vector3d ai =
        (q.col(i + 2) - 2 * q.col(i + 1) + q.col(i)) / (ts * ts);

**并非数值等价实现。** 当前源码默认未定义 SECOND_DERIVATIVE_CONTINOUS，使用其 else 分支：分别对 XYZ 轴的正/负超限量作平方惩罚，且速度项另乘 $1/ts^2$。可概括为：

$$
J_{f,\mathrm{code}}=
\frac{1}{\Delta t^2}\sum_{k,d}\max(|V_{k,d}|-v_m,0)^2
+\sum_{k,d}\max(|A_{k,d}|-a_m,0)^2.
$$

例如 $V=(0.8,0.8,0)$、$v_m=1$ 时，各轴均不超限，但范数约为 1.13，论文的范数惩罚与逐轴惩罚会给出不同结果。

结果还经过 UniformBspline::checkFeasibility() 的导数控制点逐轴检查，以及管理器 checkDynamicFeasibility() 的速度/加速度范数采样检查。后者使用名义限值加容差，是独立检查，不能据此把上述优化代价说成与论文完全相同。

<a id="eq-14"></a>
## 式 (14)：A* 候选 XY 位置的投影比例

**论文：第 5 页，V-B。**

$$
\beta(x_{xy})=
\Pi_{[0,1]}
\left(
\frac{(x_{xy}-x^{in}_{xy})^\top(x^{out}_{xy}-x^{in}_{xy})}
{\|x^{out}_{xy}-x^{in}_{xy}\|^2}
\right).
$$

来源：[dyn_a_star.cpp](../src/planner/path_searching/src/dyn_a_star.cpp)，AstarSearch() 内的 interpolateZIndexOnSearchPlane。

    double ratio =
        (sample_xy - search_start_xy).dot(search_xy_delta) / search_xy_len2;
    ratio = std::max(0.0, std::min(1.0, ratio));

ratio 对应 $\beta$，search_xy_delta 为出入口 XY 差，search_xy_len2 为其平方长度。第二句实现未单独编号的投影算子 $\Pi_{[0,1]}(\eta)=\min(1,\max(0,\eta))$。

代码采用已经转换、可能经占用调整的搜索起终点。水平起终点几乎重合时直接使用起点 Z 索引。注意：这里按起终点直线的投影比例计算，而式 (7)、(8) 使用整条候选折线的累计路程比例，两者不同。

<a id="eq-15"></a>
## 式 (15)：A* 搜索面的高度

**论文：第 5 页，V-B。**

$$
z(x_{xy})=z_{in}+\beta(x_{xy})(z_{out}-z_{in}).
$$

来源：[dyn_a_star.cpp](../src/planner/path_searching/src/dyn_a_star.cpp)，同一个 interpolateZIndexOnSearchPlane。

    const double z = search_start(2)
        + ratio * (search_end(2) - search_start(2));
    return static_cast<int>((z - center_(2)) * inv_step_size_ + 0.5)
        + CENTER_IDX_(2);

第一句与论文高度插值对应；第二句将世界高度转成离散搜索池 Z 索引。邻居扩展仅枚举 dx、dy，再调用该函数给定 Z，所以并不是自由扩展 dz 的三维 26 邻域搜索。

## 正文中没有编号、但阅读时也应对照的关系

- **轨迹时长 $T=(N_c-p_b)\Delta t$**：均匀样条节点下，对照 UniformBspline 的节点生成、getTimeSpan() 和 getTimeSum()。节点时间拉伸后要以实际节点向量求时长，不能机械沿用单一名义间隔。
- **优化的内部控制点范围**：rebound_optimize() 的 start_id=order_、end_id=cps_.size-order_；只将该区间复制成求解变量，首尾各三个控制点固定。
- **$\rho$、F、$\Pi$ 的具体实现**：分别见式 (10)、(13)、(14)，它们不是额外编号公式。
- **高度梯度抑制**：两个 combineCost 函数中的 grad_3D.row(2).setZero()。这让实际更新主要在水平面内，区别于任意三维自由位置优化。
- **多项式参考与初值**：planGlobalTraj*() 和 one_segment_traj_gen()/minSnapTraj() 提供连续参考或候选，论文没有展开这些系数线性系统的全部推导；不应将它们强行标成式 (1) 的实现。
- **地图与死胡同恢复**：Fig. 5 的滑动地图没有独立编号公式；V-C 的虚拟边界层恢复在当前主调用链中尚未完整定位，应与普通随机重试、目标调整区分。

## 建议的对照阅读路线

先读式 (7)、(8) 的高度例子，再读式 (2)～(5) 的朝向与碰撞模型；接着沿式 (14)、(15) 理解 A* 引导；最后读式 (9)～(11) 如何把引导变成梯度，并回到式 (6)、(12)、(13) 理解优化目标。式 (1) 是理解速度、加速度和 jerk 的共同基础，可在读取各项代价时反复查阅。
