/**
 * @file util.h
 * @author Marc-Philip Ecker
 * @date 13.01.20
 */
#ifndef SRC_UTIL_H
#define SRC_UTIL_H

#include <Eigen/Dense>

Eigen::Matrix4d quaternion_left(const Eigen::Quaterniond &q);

Eigen::Matrix4d quaternion_right(const Eigen::Quaterniond &q);

Eigen::Matrix3d skew(const Eigen::Vector3d &a);

#endif //SRC_UTIL_H
