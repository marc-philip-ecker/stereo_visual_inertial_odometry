/**
 * @file imu_cost_function.h
 * @author Marc-Philip Ecker
 * @date 12.01.20
 */
#ifndef SRC_IMU_COST_FUNCTION_H
#define SRC_IMU_COST_FUNCTION_H

#include <ceres/ceres.h>
#include <stereo_vio/model/control_input.h>

class ImuCostFunction : public ceres::SizedCostFunction<15, 3, 3, 4, 3, 3, 3, 3, 4, 3, 3>
{
public:
    ImuCostFunction(ControlInput u);

    bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const override;

private:
    ControlInput u_;

    Eigen::Matrix<double, 15, 15> information_sqrt_;

    Eigen::Vector3d g_;

    double dt_;

};
#endif //SRC_IMU_COST_FUNCTION_H
