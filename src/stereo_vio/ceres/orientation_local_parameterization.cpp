/**
 * @file orientation_local_parameterization.cpp
 * @author Marc-Philip Ecker
 * @date 20.12.19
 */
#include "stereo_vio/ceres/orientation_local_parameterization.h"

bool OrientationLocalParameterization::Plus(const double *x, const double *delta, double *x_plus_delta) const
{
    const Eigen::Map<const Eigen::Quaterniond> eigen_x(x);
    const Eigen::Map<const Eigen::Vector3d> eigen_delta(delta);
    Eigen::Map <Eigen::Quaterniond> eigen_x_plus_delta(x_plus_delta);

    double delta_norm = eigen_delta.norm();
    if (delta_norm > 0.0)
    {
        const Eigen::Vector3d delta_normalized = eigen_delta / delta_norm;

        Eigen::Quaterniond dq;
        dq.w() = cos(0.5 * delta_norm);
        dq.vec() = delta_normalized * sin(0.5 * delta_norm);

        eigen_x_plus_delta = dq * eigen_x;
    }
    else
    {
        eigen_x_plus_delta = eigen_x;
    }

    return true;
}

bool OrientationLocalParameterization::ComputeJacobian(const double *x, double *jacobian) const
{
    Eigen::Map <Eigen::Matrix<double, 4, 3, Eigen::RowMajor>> J(jacobian);

    J.block<3, 3>(0, 0).setIdentity();
    J.block<1, 3>(3, 0).setZero();

    return true;
}

int OrientationLocalParameterization::GlobalSize() const
{
    return 4;
}

int OrientationLocalParameterization::LocalSize() const
{
    return 3;
}