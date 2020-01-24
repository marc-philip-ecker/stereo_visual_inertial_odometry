/**
 * @file orientation_local_parameterization.h
 * @author Marc-Philip Ecker
 * @date 20.12.19
 */
#ifndef SRC_ORIENTATION_LOCAL_PARAMETERIZATION_H
#define SRC_ORIENTATION_LOCAL_PARAMETERIZATION_H

#include <ceres/ceres.h>

class OrientationLocalParameterization : public ceres::LocalParameterization
{
public:
    bool Plus(const double *x, const double *delta, double *x_plus_delta) const override;

    bool ComputeJacobian(const double *x, double *jacobian) const override;

    int GlobalSize() const override;

    int LocalSize() const override;
};

#endif //SRC_ORIENTATION_LOCAL_PARAMETERIZATION_H