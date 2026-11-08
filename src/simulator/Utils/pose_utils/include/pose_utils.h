/// @file
/// Armadillo 位姿工具声明，主要为 odom_visualization 提供旋转和四元数转换。
/// 角度单位 rad，三维位姿采用位置加 yaw/pitch/roll，四元数向量按 [w,x,y,z] 排列。
#ifndef POSE_UTILS_H
#define POSE_UTILS_H

#include <iostream>
#include "armadillo"

#define PI 3.14159265359
#define NUM_INF 999999.9

using namespace arma;
using namespace std;

// Rotation ---------------------
mat ypr_to_R(const colvec& ypr);

mat yaw_to_R(double yaw);

colvec R_to_ypr(const mat& R);

mat quaternion_to_R(const colvec& q);

colvec R_to_quaternion(const mat& R);

colvec quaternion_mul(const colvec& q1, const colvec& q2);

colvec quaternion_inv(const colvec& q);

// General Pose Update ----------
colvec pose_update(const colvec& X1, const colvec& X2);

colvec pose_inverse(const colvec& X);

colvec pose_update_2d(const colvec& X1, const colvec& X2);

colvec pose_inverse_2d(const colvec& X);

// For Pose EKF -----------------
mat Jplus1(const colvec& X1, const colvec& X2);

mat Jplus2(const colvec& X1, const colvec& X2);

// For IMU EKF ------------------
colvec state_update(const colvec& X, const colvec& U, double dt);

mat jacobianF(const colvec& X, const colvec& U, double dt);

mat jacobianU(const colvec& X, const colvec& U, double dt);

colvec state_measure(const colvec& X);

mat jacobianH();

#endif
