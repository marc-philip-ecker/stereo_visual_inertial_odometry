/**
 * @file util.cpp
 * @author Marc-Philip Ecker
 * @date 13.01.20
 */
#include "stereo_vio/processing/util.h"

Eigen::Matrix4d quaternion_left(const Eigen::Quaterniond &q)
{
    return (Eigen::Matrix4d()
            << q.w(), -q.z(), q.y(), q.x(),
            q.z(), q.w(), -q.x(), q.y(),
            -q.y(), q.x(), q.w(), q.z(),
            -q.x(), -q.y(), -q.z(), q.w()).finished();
}

Eigen::Matrix4d quaternion_right(const Eigen::Quaterniond &q)
{
    return (Eigen::Matrix4d()
            << q.w(), q.z(), -q.y(), q.x(),
            -q.z(), q.w(), q.x(), q.y(),
            q.y(), -q.x(), q.w(), q.z(),
            -q.x(), -q.y(), -q.z(), q.w()).finished();
}

Eigen::Matrix3d skew(const Eigen::Vector3d &a)
{
    return (Eigen::Matrix3d()
            << 0, -a.z(), a.y(),
            a.z(), 0, -a.x(),
            -a.y(), a.x(), 0).finished();
}