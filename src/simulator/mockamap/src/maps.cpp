/// @file
/// 多种仿真场景的几何生成器，Maps::generate() 按 type 选择具体算法。
/// 默认 type=2 为随机箱体表面点云，固定 seed 便于同一实现和环境下复现；空间单位由 scale 转换。
/// 场景点云是仿真环境真值，局部规划地图需要通过模拟观测另行建立。
#include "maps.hpp"

#include <algorithm>
#include <iostream>
#include <random>
#include <vector>

#include <Eigen/Core>

#include "perlinnoise.hpp"

using namespace mocka;

/// @brief 按固定种子抽取箱体位置、宽度和高度，在侧面、底面和顶面按 surface_resolution 采样。
void
Maps::randomMapGenerate()
{

  std::default_random_engine eng(info.seed);

  double _resolution = 1 / info.scale;

  double _x_l = -info.sizeX / (2 * info.scale);
  double _x_h = info.sizeX / (2 * info.scale);
  double _y_l = -info.sizeY / (2 * info.scale);
  double _y_h = info.sizeY / (2 * info.scale);
  double _h_l = 0;
  double _h_h = info.sizeZ / info.scale;

  double _w_l, _w_h;
  int    _ObsNum;

  // 本段读取启动配置：下方“正常默认”来自当前 launch/XML，实际运行以参数服务器为准。
  // param() 的末项仅是缺参回退；getParam()/必需参数封装没有这一回退，具体见各行说明。
  // 参数读取：随机箱体底面宽度的抽样下限，单位 m。
  // 配置来源：simulator.xml → /mockamap_node/width_min；正常默认启动取 0.2。
  // 缺参回退：0.6；只有参数服务器中没有该键时才使用。
  info.nh_private->param("width_min", _w_l, 0.6);
  // 参数读取：随机箱体底面宽度的抽样上限，单位 m；抽样中心和宽度共同决定表面位置。
  // 配置来源：simulator.xml → /mockamap_node/width_max；正常默认启动取 0.8。
  // 缺参回退：1.5；只有参数服务器中没有该键时才使用。
  info.nh_private->param("width_max", _w_h, 1.5);
  // 参数读取：随机箱体高度抽样下限，单位 m；当前与上限同为 2，生成等高箱体。
  // 配置来源：simulator.xml → /mockamap_node/height_min；正常默认启动取 2.0。
  // 缺参回退：_h_l；只有参数服务器中没有该键时才使用。
  info.nh_private->param("height_min", _h_l, _h_l);
  // 参数读取：随机箱体高度抽样上限，单位 m；生成器还按地图高度限制有效范围。
  // 配置来源：simulator.xml → /mockamap_node/height_max；正常默认启动取 2.0。
  // 缺参回退：_h_h；只有参数服务器中没有该键时才使用。
  info.nh_private->param("height_max", _h_h, _h_h);
  // 参数读取：随机箱体抽样数量，不等于点云点数；箱体相交时最终独立可见障碍数也可能不同。
  // 配置来源：simulator.xml → /mockamap_node/obstacle_number；正常默认启动取 500。
  // 缺参回退：10；只有参数服务器中没有该键时才使用。
  info.nh_private->param("obstacle_number", _ObsNum, 10);
  double surface_resolution;
  // 参数读取：箱体表面采样间距的目标尺度，单位 m；越小通常生成越多场景点。
  // 配置来源：simulator.xml → /mockamap_node/surface_resolution；正常默认启动取 0.05。
  // 缺参回退：_resolution * 0.5；只有参数服务器中没有该键时才使用。
  info.nh_private->param("surface_resolution", surface_resolution, _resolution * 0.5);

  _h_l = std::max(0.0, _h_l);
  _h_h = std::min(info.sizeZ / info.scale, std::max(_h_l, _h_h));
  surface_resolution = std::max(1e-3, surface_resolution);

  std::uniform_real_distribution<double> rand_x;
  std::uniform_real_distribution<double> rand_y;
  std::uniform_real_distribution<double> rand_w;
  std::uniform_real_distribution<double> rand_h;

  pcl::PointXYZ pt_random;
  auto pushPoint = [&](double px, double py, double pz) {
    pt_random.x = px;
    pt_random.y = py;
    pt_random.z = pz;
    info.cloud->points.push_back(pt_random);
  };

  rand_x = std::uniform_real_distribution<double>(_x_l, _x_h);
  rand_y = std::uniform_real_distribution<double>(_y_l, _y_h);
  rand_w = std::uniform_real_distribution<double>(_w_l, _w_h);
  rand_h = std::uniform_real_distribution<double>(_h_l, _h_h);

  for (int i = 0; i < _ObsNum; i++)
  {
    double x, y;
    x = rand_x(eng);
    y = rand_y(eng);

    double w, h;
    w = rand_w(eng);
    h = rand_h(eng);

    const double half_w = w * 0.5;
    const double x_min = x - half_w;
    const double x_max = x + half_w;
    const double y_min = y - half_w;
    const double y_max = y + half_w;
    const int xy_steps = std::max(1, static_cast<int>(std::ceil(w / surface_resolution)));
    const int z_steps = std::max(1, static_cast<int>(std::ceil(h / surface_resolution)));

    for (int iz = 0; iz <= z_steps; ++iz)
    {
      const double z = h * iz / z_steps;
      for (int ix = 0; ix <= xy_steps; ++ix)
      {
        const double px = x_min + w * ix / xy_steps;
        pushPoint(px, y_min, z);
        pushPoint(px, y_max, z);
      }
      for (int iy = 0; iy <= xy_steps; ++iy)
      {
        const double py = y_min + w * iy / xy_steps;
        pushPoint(x_min, py, z);
        pushPoint(x_max, py, z);
      }
    }

    for (int ix = 0; ix <= xy_steps; ++ix)
      for (int iy = 0; iy <= xy_steps; ++iy)
      {
        const double px = x_min + w * ix / xy_steps;
        const double py = y_min + w * iy / xy_steps;
        pushPoint(px, py, 0.0);
        pushPoint(px, py, h);
      }
  }

  info.cloud->width    = info.cloud->points.size();
  info.cloud->height   = 1;
  info.cloud->is_dense = true;

  pcl2ros();
}

/// @brief 把几何点云转换为世界系 PointCloud2 并写入调用方输出对象。
void
Maps::pcl2ros()
{
  pcl::toROSMsg(*info.cloud, *info.output);
  info.output->header.frame_id = "world";
  ROS_INFO("finish: infill %lf%%",
           info.cloud->width / (1.0 * info.sizeX * info.sizeY * info.sizeZ));
}

/// @brief 读取复杂度、阈值和分形参数，按三维噪声值生成障碍几何，默认 type=2 不执行此分支。
void
Maps::perlin3D()
{
  double complexity;
  double fill;
  int    fractal;
  double attenuation;

  // 参数读取：仅 Perlin 分支使用的空间噪声变化尺度；默认 type 2 不使用它。
  // 配置来源：simulator.xml → /mockamap_node/complexity；正常默认启动取 0.05。
  // 缺参回退：0.142857；只有参数服务器中没有该键时才使用。
  info.nh_private->param("complexity", complexity, 0.142857);
  // 参数读取：Perlin 分支选择噪声分位阈值的填充比例，默认 type 2 不使用它。
  // 配置来源：simulator.xml → /mockamap_node/fill；正常默认启动取 0.12。
  // 缺参回退：0.38；只有参数服务器中没有该键时才使用。
  info.nh_private->param("fill", fill, 0.38);
  // 参数读取：Perlin 分支叠加的噪声层数，正整数；默认 type 2 不使用它。
  // 配置来源：simulator.xml → /mockamap_node/fractal；正常默认启动取 1。
  // 缺参回退：1；只有参数服务器中没有该键时才使用。
  info.nh_private->param("fractal", fractal, 1);
  // 参数读取：Perlin 分支各层幅度的缩放系数，具体按 attenuation/层序号计算。
  // 配置来源：simulator.xml → /mockamap_node/attenuation；正常默认启动取 0.1。
  // 缺参回退：0.5；只有参数服务器中没有该键时才使用。
  info.nh_private->param("attenuation", attenuation, 0.5);

  info.cloud->width  = info.sizeX * info.sizeY * info.sizeZ;
  info.cloud->height = 1;
  info.cloud->points.resize(info.cloud->width * info.cloud->height);

  PerlinNoise noise(info.seed);

  std::vector<double>* v = new std::vector<double>;
  v->reserve(info.cloud->width);
  for (int i = 0; i < info.sizeX; ++i)
  {
    for (int j = 0; j < info.sizeY; ++j)
    {
      for (int k = 0; k < info.sizeZ; ++k)
      {
        double tnoise = 0;
        for (int it = 1; it <= fractal; ++it)
        {
          int    dfv = pow(2, it);
          double ta  = attenuation / it;
          tnoise += ta * noise.noise(dfv * i * complexity,
                                     dfv * j * complexity,
                                     dfv * k * complexity);
        }
        v->push_back(tnoise);
      }
    }
  }
  std::sort(v->begin(), v->end());
  int    tpos = info.cloud->width * (1 - fill);
  double tmp  = v->at(tpos);
  ROS_INFO("threshold: %lf", tmp);

  int pos = 0;
  for (int i = 0; i < info.sizeX; ++i)
  {
    for (int j = 0; j < info.sizeY; ++j)
    {
      for (int k = 0; k < info.sizeZ; ++k)
      {
        double tnoise = 0;
        for (int it = 1; it <= fractal; ++it)
        {
          int    dfv = pow(2, it);
          double ta  = attenuation / it;
          tnoise += ta * noise.noise(dfv * i * complexity,
                                     dfv * j * complexity,
                                     dfv * k * complexity);
        }
        if (tnoise > tmp)
        {
          info.cloud->points[pos].x =
            i / info.scale - info.sizeX / (2 * info.scale);
          info.cloud->points[pos].y =
            j / info.scale - info.sizeY / (2 * info.scale);
          info.cloud->points[pos].z = k / info.scale;
          pos++;
        }
      }
    }
  }
  info.cloud->width = pos;
  ROS_INFO("the number of points before optimization is %d", info.cloud->width);
  info.cloud->points.resize(info.cloud->width * info.cloud->height);
  pcl2ros();
}

/// @brief 递归划分指定二维网格区域并设置通道，用于迷宫结构构造。
void
Maps::recursiveDivision(int xl, int xh, int yl, int yh, Eigen::MatrixXi& maze)
{
  ROS_INFO(
    "generating maze with width %d , height %d", xh - xl + 1, yh - yl + 1);

  if (xl < xh - 3 && yl < yh - 3)
  { // the remaining area is larger than or equal to 5*5, need to add both x
    // wall and y wall
    bool valid = false; // used to judge whether the wall selection is valid
    int  xm    = 0;
    int  ym    = 0;
    ROS_INFO("entered 5*5 mode");
    while (valid == false)
    {
      xm = (std::rand() % (xh - xl - 1) + xl +
            1); // generating random number between xl+1 and xh-1(pointless to
                // add a wall at the sides)
      ym = (std::rand() % (yh - yl - 1) + yl +
            1); // generating random number between yl+1 and yh-1(pointless to
                // add a wall at the sides)
      if (xl - 1 >= 0)
      { // there is a point at xl-1,ym
        if (maze(xl - 1, ym) == 0)
        { // this is an opening,need to change random number
          continue;
        }
      }

      else if (xh + 1 <= maze.cols() - 1)
      { // there is a point at xh+1,ym
        if (maze(xh + 1, ym) == 0)
        { // this is an opening,need to change random number
          continue;
        }
      }

      else if (yl - 1 >= 0)
      { // there is a point at xm,yl-1
        if (maze(xm, yl - 1) == 0)
        { // this is an opening,need to change random number
          continue;
        }
      }

      else if (yh + 1 <= maze.rows() - 1)
      { // there is a point at xm,yh+1
        if (maze(xm, yh + 1) == 0)
        { // this is an opening,need to change random number
          continue;
        }
      }

      valid = true;

    } // xm and ym are now the valid coordinate of the center of the wall
    for (int i = xl; i <= xh; i++)
    {
      maze(i, ym) = 1;
    }
    for (int j = yl; j <= yh; j++)
    {
      maze(xm, j) = 1;
    } // adding walls around the center point
    int d1 = std::rand() % (xm - xl) + xl;
    int d2 = std::rand() % (xh - xm) + xm + 1;
    int d3 = std::rand() % (ym - yl) + yl;
    int d4 =
      std::rand() % (yh - ym) + ym + 1; // generating four possible door points

    int decision = std::rand() % 4; // random selection of three doors
    switch (decision)
    {
      case 0:
        maze(d1, ym) = 0;
        maze(d2, ym) = 0;
        maze(xm, d3) = 0;
        break;

      case 1:
        maze(d1, ym) = 0;
        maze(d2, ym) = 0;
        maze(xm, d4) = 0;
        break;

      case 2:
        maze(d2, ym) = 0;
        maze(xm, d3) = 0;
        maze(xm, d4) = 0;
        break;

      case 3:
        maze(d1, ym) = 0;
        maze(xm, d3) = 0;
        maze(xm, d4) = 0;
        break;
    } // the doors are opened for this cell
    if (yl - 1 >= 0)
    {
      if (maze(xm, yl - 1) == 0)
      {
        maze(xm, yl) = 0;
      }
    }

    if (yh + 1 <= maze.rows() - 1)
    {
      if (maze(xm, yh + 1) == 0)
      {
        maze(xm, yh) = 0;
      }
    }

    if (xl - 1 >= 0)
    {
      if (maze(xl - 1, ym) == 0)
      {
        maze(xl, ym) = 0;
      }
    }

    if (xh + 1 <= maze.cols() - 1)
    {
      if (maze(xh + 1, ym) == 0)
      {
        maze(xh, ym) = 0;
      }
    }

    std::cout << maze << std::endl;
    recursiveDivision(xl, xm - 1, yl, ym - 1, maze);
    recursiveDivision(xm + 1, xh, yl, ym - 1, maze);
    recursiveDivision(xl, xm - 1, ym + 1, yh, maze);
    recursiveDivision(xm + 1, xh, ym + 1, yh, maze);

    ROS_INFO("finished generating maze with width %d , height %d",
             xh - xl + 1,
             yh - yl + 1);
    std::cout << maze << std::endl;
    return;
  } // when the remaining area is larger than or equal to 5*5

  else if (xl < xh - 2 && yl < yh - 2)
  {
    // bool valid     = false; // used to judge whether the wall selection is valid
    int  xm        = 0;
    int  ym        = 0;
    int  doorcount = 0;
    xm             = (std::rand() % (xh - xl - 1) + xl +
          1); // generating random number between xl+1 and xh-1(pointless to
                          // add a wall at the sides)
    ym =
      (std::rand() % (yh - yl - 1) + yl +
       1); // generating random number between yl+1 and yh-1(pointless to
           // add a wall at the sides)
           // xm and ym are now the valid coordinate of the center of the wall
    for (int i = xl; i <= xh; i++)
    {
      maze(i, ym) = 1;
    }
    for (int j = yl; j <= yh; j++)
    {
      maze(xm, j) = 1;
    } // adding walls around the center point
    if (yl - 1 >= 0)
    {
      if (maze(xm, yl - 1) == 0)
      {
        maze(xm, yl) = 0;
        doorcount++;
      }
    }

    if (yh + 1 <= maze.rows() - 1)
    {
      if (maze(xm, yh + 1) == 0)
      {
        maze(xm, yh) = 0;
        doorcount++;
      }
    }

    if (xl - 1 >= 0)
    {
      if (maze(xl - 1, ym) == 0)
      {
        maze(xl, ym) = 0;
        doorcount++;
      }
    }

    if (xh + 1 <= maze.cols() - 1)
    {
      if (maze(xh + 1, ym) == 0)
      {
        maze(xh, ym) = 0;
        doorcount++;
      }
    }

    int d1 = std::rand() % (xm - xl) + xl;
    int d2 = std::rand() % (xh - xm) + xm + 1;
    int d3 = std::rand() % (ym - yl) + yl;
    int d4 =
      std::rand() % (yh - ym) + ym + 1; // generating four possible door points

    int decision = std::rand() % 4; // random selection of three doors
    switch (decision)
    {
      case 0:
        maze(d1, ym) = 0;
        maze(d2, ym) = 0;
        maze(xm, d3) = 0;
        break;

      case 1:
        maze(d1, ym) = 0;
        maze(d2, ym) = 0;
        maze(xm, d4) = 0;
        break;

      case 2:
        maze(d2, ym) = 0;
        maze(xm, d3) = 0;
        maze(xm, d4) = 0;
        break;

      case 3:
        maze(d1, ym) = 0;
        maze(xm, d3) = 0;
        maze(xm, d4) = 0;
        break;
    } // the doors are opened for this cell
    std::cout << maze << std::endl;

    ROS_INFO("finished generating maze with width %d , height %d",
             xh - xl + 1,
             yh - yl + 1);
    std::cout << maze << std::endl;
    return;
  }

  else if (xl < xh - 1 && yl < yh - 2)
  { // the case of 3*4+
    ROS_INFO("entered 3*4+ mode");
    int doorcount = 0;
    int ym        = 0;
    for (int i = yl; i <= yh; i++)
    {
      maze(xl + 1, i) = 1;
    } // filling a center wall
    if (yl - 1 >= 0)
    {
      if (maze(xl + 1, yl - 1) == 0)
      {
        maze(xl + 1, yl) = 0;
        doorcount++;
      }
    }
    if (yh + 1 <= maze.rows() - 1)
    {
      if (maze(xl + 1, yh + 1) == 0)
      {
        maze(xl + 1, yh) = 0;
        doorcount++;
      }
    } // opening doors if the wall blocks the old doors
    if (doorcount == 0)
    {
      ym               = std::rand() % (yh - yl + 1) + yl;
      maze(xl + 1, ym) = 0;
    }
  } // the case of 4+*3
  //
  else if (xl < xh - 2 && yl < yh - 1)
  { // the case of 4+*3
    ROS_INFO("entered 4+*3 mode");
    int doorcount = 0;
    int xm        = 0;
    for (int i = xl; i <= xh; i++)
    {
      maze(i, yl + 1) = 1;
    } // filling a center wall
    if (xl - 1 >= 0)
    {
      if (maze(xl - 1, yl + 1) == 0)
      {
        maze(xl, yl + 1) = 0;
        doorcount++;
      }
    }
    if (xh + 1 <= maze.cols() - 1)
    {
      if (maze(xh + 1, yl + 1) == 0)
      {
        maze(xh, yl + 1) = 0;
        doorcount++;
      }
    } // opening doors if the wall blocks the old doors
    if (doorcount == 0)
    {
      xm               = std::rand() % (xh - xl + 1) + xl;
      maze(xm, yl + 1) = 0;
    }
  } // the case of 4+*3

  else if (xl < xh - 1 && yl < yh - 1)
  { // the case of 3*3
    maze(xl + 1, yl + 1) = 1;
    return;
  }
  else
  {
    ROS_INFO("finished generating maze with width %d , height %d",
             xh - xl + 1,
             yh - yl + 1);
    return;
  }
}

/// @brief 为迷宫网格组织递归分割过程，保留原函数拼写。
void
Maps::recursizeDivisionMaze(Eigen::MatrixXi& maze)
{
  //! @todo all bugs here...
  int sx = maze.rows();
  int sy = maze.cols();

  int px, py;

  if (sx > 5)
    px = (std::rand() % (sx - 3) + 1);
  else
    return;

  if (sy > 5)
    py = (std::rand() % (sy - 3) + 1);
  else
    return;

  ROS_INFO("debug %d %d %d %d", sx, sy, px, py);

  int x1, x2, y1, y2;

  if (px != 1)
    x1 = (std::rand() % (px - 1) + 1);
  else
    x1 = 1;

  if ((sx - px - 3) > 0)
    x2 = (std::rand() % (sx - px - 3) + px + 1);
  else
    x2 = px + 1;

  if (py != 1)
    y1 = (std::rand() % (py - 1) + 1);
  else
    y1 = 1;

  if ((sy - py - 3) > 0)
    y2 = (std::rand() % (sy - py - 3) + py + 1);
  else
    y2 = py + 1;
  ROS_INFO("%d %d %d %d", x1, x2, y1, y2);

  if (px != 1 && px != (sx - 2))
  {
    for (int i = 1; i < (sy - 1); ++i)
    {
      if (i != y1 && i != y2)
        maze(px, i) = 1;
    }
  }
  if (py != 1 && py != (sy - 2))
  {
    for (int i = 1; i < (sx - 1); ++i)
    {
      if (i != x1 && i != x2)
        maze(i, py) = 1;
    }
  }
  switch (std::rand() % 4)
  {
    case 0:
      maze(x1, py) = 1;
      break;
    case 1:
      maze(x2, py) = 1;
      break;
    case 2:
      maze(px, y1) = 1;
      break;
    case 3:
      maze(px, y2) = 1;
      break;
  }

  if (px > 2 && py > 2)
  {
    Eigen::MatrixXi sub = maze.block(0, 0, px + 1, py + 1);
    recursizeDivisionMaze(sub);
    maze.block(0, 0, px, py) = sub;
  }
  if (px > 2 && (sy - py - 1) > 2)
  {
    Eigen::MatrixXi sub = maze.block(0, py, px + 1, sy - py);
    recursizeDivisionMaze(sub);
    maze.block(0, py, px + 1, sy - py) = sub;
  }
  if (py > 2 && (sx - px - 1) > 2)
  {
    Eigen::MatrixXi sub = maze.block(px, 0, sx - px, py + 1);
    recursizeDivisionMaze(sub);
    maze.block(px, 0, sx - px, py + 1) = sub;
  }
  if ((sx - px - 1) > 2 && (sy - py - 1) > 2)
  {

    Eigen::MatrixXi sub = maze.block(px, py, sy - px, sy - py);

    recursizeDivisionMaze(sub);
    maze.block(px, py, sy - px, sy - py) = sub;
  }
}

/// @brief 将二维迷宫网格转成可发布的三维障碍点云。
void
Maps::maze2D()
{
  double width;
  int    type;
  int    addWallX;
  int    addWallY;
  // 参数读取：仅二维迷宫分支使用的通路宽度（m），参与迷宫离散格数计算。
  // 配置位置：私有 /mockamap_node/road_width；当前主 launch 未覆盖此键，取下面的缺参回退。
  // 缺参回退：1.0；只有参数服务器中没有该键时才使用。
  info.nh_private->param("road_width", width, 1.0);
  // 参数读取：仅二维迷宫分支使用的 x 方向外围墙开关。
  // 配置位置：私有 /mockamap_node/add_wall_x；当前主 launch 未覆盖此键，取下面的缺参回退。
  // 缺参回退：0；只有参数服务器中没有该键时才使用。
  info.nh_private->param("add_wall_x", addWallX, 0);
  // 参数读取：仅二维迷宫分支使用的 y 方向外围墙开关。
  // 配置位置：私有 /mockamap_node/add_wall_y；当前主 launch 未覆盖此键，取下面的缺参回退。
  // 缺参回退：0；只有参数服务器中没有该键时才使用。
  info.nh_private->param("add_wall_y", addWallY, 0);
  // 参数读取：二维迷宫内部生成方式选择；仅 Maps::maze2D() 读取。
  // 配置位置：私有 /mockamap_node/maze_type；当前主 launch 未覆盖此键，取下面的缺参回退。
  // 缺参回退：1；只有参数服务器中没有该键时才使用。
  info.nh_private->param("maze_type", type, 1);

  int mx = info.sizeX / (width * info.scale);
  int my = info.sizeY / (width * info.scale);

  Eigen::MatrixXi maze(mx, my);
  maze.setZero();

  switch (type)
  {
    case 1:
      recursiveDivision(0, maze.cols() - 1, 0, maze.rows() - 1, maze);
      break;
  }

  if (addWallX)
  {
    for (int i = 0; i < mx; ++i)
    {
      maze(i, 0)      = 1;
      maze(i, my - 1) = 1;
    }
  }
  if (addWallY)
  {
    for (int i = 0; i < my; ++i)
    {
      maze(0, i)      = 1;
      maze(mx - 1, i) = 1;
    }
  }

  std::cout << maze << std::endl;

  for (int i = 0; i < mx; ++i)
  {
    for (int j = 0; j < my; ++j)
    {
      if (maze(i, j))
      {
        for (int ii = 0; ii < width * info.scale; ++ii)
        {
          for (int jj = 0; jj < width * info.scale; ++jj)
          {
            for (int k = 0; k < info.sizeZ; ++k)
            {
              pcl::PointXYZ pt_random;
              pt_random.x =
                i * width + ii / info.scale - info.sizeX / (2.0 * info.scale);
              pt_random.y =
                j * width + jj / info.scale - info.sizeY / (2.0 * info.scale);
              pt_random.z = k / info.scale;
              info.cloud->points.push_back(pt_random);
            }
          }
        }
      }
    }
  }
  info.cloud->width    = info.cloud->points.size();
  info.cloud->height   = 1;
  info.cloud->is_dense = true;
  pcl2ros();
}

Maps::BasicInfo
/// @brief 返回生成器配置副本。
Maps::getInfo() const
{
  return info;
}

/// @brief 接收场景尺寸、种子、参数句柄和输出存储指针。
void
Maps::setInfo(const BasicInfo& value)
{
  info = value;
}

Maps::Maps()
{
}

/// @brief 场景分派：1=Perlin，2=随机箱体，3=二维迷宫，4=三维迷宫；完成后由主节点重复发布。
void
Maps::generate(int type)
{
  switch (type)
  {
    default:
    case 1:
      perlin3D();
      break;
    case 2:
      randomMapGenerate();
      break;
    case 3:
      std::srand(info.seed);
      maze2D();
      break;
    case 4: // generating 3d maze
      std::srand(info.seed);
      Maze3DGen();
      break;
  }
}

pcl::PointXYZ
MazePoint::getPoint()
{
  return point;
}

int
MazePoint::getPoint1()
{
  return point1;
}

int
MazePoint::getPoint2()
{
  return point2;
}

double
MazePoint::getDist1()
{
  return dist1;
}

double
MazePoint::getDist2()
{
  return dist2;
}

void
MazePoint::setPoint(pcl::PointXYZ p)
{
  point = p;
}

void
MazePoint::setPoint1(int p)
{
  point1 = p;
}

void
MazePoint::setPoint2(int p)
{
  point2 = p;
}

void
MazePoint::setDist1(double set)
{
  dist1 = set;
}

void
MazePoint::setDist2(double set)
{
  dist2 = set;
}

/// @brief 构造三维迷宫及其连接关系，再转换为点云，属于可选场景。
void
Maps::Maze3DGen()
{
  // getting required info parameters from the given node
  int    numNodes;
  double connectivity;
  int    nodeRad;
  int    roadRad;

  // 参数读取：三维迷宫的基础节点数量，仅 type 4 场景读取。
  // 配置位置：私有 /mockamap_node/numNodes；当前主 launch 未覆盖此键，取下面的缺参回退。
  // 缺参回退：10；只有参数服务器中没有该键时才使用。
  info.nh_private->param("numNodes", numNodes, 10);
  // 参数读取：三维迷宫连通相关比例，仅 type 4 读取，具体选择过程见后续几何生成。
  // 配置位置：私有 /mockamap_node/connectivity；当前主 launch 未覆盖此键，取下面的缺参回退。
  // 缺参回退：0.5；只有参数服务器中没有该键时才使用。
  info.nh_private->param("connectivity", connectivity, 0.5);
  // 参数读取：三维迷宫节点几何的离散半径参数，按本生成器的尺度转成场景点，不是机器人半径。
  // 配置位置：私有 /mockamap_node/nodeRad；当前主 launch 未覆盖此键，取下面的缺参回退。
  // 缺参回退：3；只有参数服务器中没有该键时才使用。
  info.nh_private->param("nodeRad", nodeRad, 3);
  // 参数读取：三维迷宫连接道路几何的离散半径参数，按本生成器尺度使用。
  // 配置位置：私有 /mockamap_node/roadRad；当前主 launch 未覆盖此键，取下面的缺参回退。
  // 缺参回退：2；只有参数服务器中没有该键时才使用。
  info.nh_private->param("roadRad", roadRad, 2);
  ROS_INFO("received parameters : numNodes: %d connectivity: "
           "%f nodeRad: %d roadRad: %d",
           numNodes,
           connectivity,
           nodeRad,
           roadRad);
  // generating random points
  std::vector<pcl::PointXYZ> base;

  for (int i = 0; i < numNodes; i++)
  {
    double rx = std::rand() / RAND_MAX +
                (std::rand() % info.sizeX) / info.scale -
                info.sizeX / (2 * info.scale);
    double ry = std::rand() / RAND_MAX +
                (std::rand() % info.sizeY) / info.scale -
                info.sizeY / (2 * info.scale);
    double rz = std::rand() / RAND_MAX +
                (std::rand() % info.sizeZ) / info.scale -
                info.sizeZ / (2 * info.scale);
    ROS_INFO("point: x: %f , y: %f , z: %f", rx, ry, rz);

    pcl::PointXYZ pt_random;
    pt_random.x = rx;
    pt_random.y = ry;
    pt_random.z = rz;
    base.push_back(pt_random);
  } // generating random cores in the space

  for (int i = 0; i < info.sizeX; i++)
  {
    for (int j = 0; j < info.sizeY; j++)
    {
      for (int k = 0; k < info.sizeZ; k++)
      { // for every scaled coordinate points
        pcl::PointXYZ test;
        test.x = i / info.scale - info.sizeX / (2 * info.scale);
        test.y = j / info.scale - info.sizeY / (2 * info.scale);
        test.z = k / info.scale -
                 info.sizeZ /
                   (2 * info.scale); // marking the corresponding point location

        MazePoint mp;
        mp.setPoint(test);
        mp.setPoint2(-1);
        mp.setPoint1(-1);
        mp.setDist1(10000.0);
        mp.setDist2(100000.0); // setting super large starting values
        for (int ii = 0; ii < numNodes; ii++)
        {
          double dist =
            std::sqrt((base[ii].x - test.x) * (base[ii].x - test.x) +
                      (base[ii].y - test.y) * (base[ii].y - test.y) +
                      (base[ii].z - test.z) * (base[ii].z - test.z));
          if (dist < mp.getDist1())
          {

            mp.setDist2(mp.getDist1());
            mp.setDist1(dist);

            mp.setPoint2(mp.getPoint1());
            mp.setPoint1(ii);
          }
          else if (dist < mp.getDist2())
          {
            mp.setDist2(dist);
            mp.setPoint2(ii);
          } // finding the distances to the nearest two cores
        }
        if (std::abs(mp.getDist2() - mp.getDist1()) < 1 / info.scale)
        { // the tested location is on one of the middle planes
          if ((mp.getPoint1() + mp.getPoint2()) >
                int((1 - connectivity) * numNodes) &&
              (mp.getPoint1() + mp.getPoint2()) <
                int((1 + connectivity) * numNodes))
          { // this is a holed wall
            double judge =
              std::sqrt((base[mp.getPoint1()].x - base[mp.getPoint2()].x) *
                          (base[mp.getPoint1()].x - base[mp.getPoint2()].x) +
                        (base[mp.getPoint1()].y - base[mp.getPoint2()].y) *
                          (base[mp.getPoint1()].y - base[mp.getPoint2()].y) +
                        (base[mp.getPoint1()].z - base[mp.getPoint2()].z) *
                          (base[mp.getPoint1()].z - base[mp.getPoint2()].z));
            if (mp.getDist1() + mp.getDist2() - judge >=
                roadRad / (info.scale * 3))
            {
              info.cloud->points.push_back(mp.getPoint());
            }
          }
          else
          {
            info.cloud->points.push_back(mp.getPoint());
          }
        }
      }
    }
  }

  info.cloud->width  = info.cloud->points.size();
  info.cloud->height = 1;
  ROS_INFO("the number of points before optimization is %d", info.cloud->width);
  info.cloud->points.resize(info.cloud->width * info.cloud->height);
  pcl2ros();
}
