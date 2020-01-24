/**
 * @file reprojection_cost_function.h
 * @author Marc-Philip Ecker
 * @date 18.12.19
 */
#ifndef SRC_REPROJECTION_COST_FUNCTION_H
#define SRC_REPROJECTION_COST_FUNCTION_H

#include <ceres/ceres.h>

class ReprojectionCostFunction : public ceres::SizedCostFunction<2, 4, 3, 3>
{
public:
    ReprojectionCostFunction(Eigen::Vector2d z, const Eigen::Matrix2d &covariance,
                             const Eigen::Quaterniond &q_body_cam, Eigen::Vector3d t_body_cam,
                             double fx, double fy, double cx, double cy);

    bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const override;

private:
    Eigen::Vector2d z_;
    Eigen::Matrix2d information_;
    Eigen::Quaterniond q_body_cam_;
    Eigen::Vector3d t_body_cam_;
    double fx_, fy_, cx_, cy_;
};

#endif //SRC_REPROJECTION_COST_FUNCTION_H