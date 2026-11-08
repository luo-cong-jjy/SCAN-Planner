// Include Files
/// @file
/// 感知视场候选块筛选接口，BoxPointType 来自 ikd-Tree 的公共几何类型。
/// 使用该类型不等于渲染主链路必然实例化增量 KD-tree；实际调用见渲染节点。
#pragma once
#include <math.h>
#include <cmath>
#include "ikd-Tree/ikd_Tree.h"
#include <Eigen/Core>
#include <algorithm>

#define eps_value 1e-6

/// @brief 保存箱体一个面的四个顶点，供 check_surface() 检查。
struct PlaneType{
    Eigen::Vector3d p[4];
};


/// @brief 保存场景边界与空间块边长，并提供从箱体到点的多层次视场判断。
class FOV_Checker{
public:
    FOV_Checker();
    ~FOV_Checker();
    void Set_Env(BoxPointType env_param);
    void Set_BoxLength(double box_len_param);
    void check_fov(Eigen::Vector3d cur_pose, Eigen::Vector3d axis, double theta, double depth, vector<BoxPointType> &boxes);
    bool check_box(Eigen::Vector3d cur_pose, Eigen::Vector3d axis, double theta, double depth, const BoxPointType box);
    bool check_line(Eigen::Vector3d cur_pose, Eigen::Vector3d axis, double theta, double depth, Eigen::Vector3d line_p, Eigen::Vector3d line_vec);
    bool check_surface(Eigen::Vector3d cur_pose, Eigen::Vector3d axis,  double theta, double depth, PlaneType plane);
    bool check_point(Eigen::Vector3d cur_pose, Eigen::Vector3d axis, double theta, double depth, Eigen::Vector3d point);
    bool check_box_in_env(BoxPointType box);    
private:
    BoxPointType env;
    double box_length;
    FILE *fp;

};
