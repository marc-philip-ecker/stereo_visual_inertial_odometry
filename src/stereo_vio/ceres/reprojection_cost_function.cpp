/**
 * @file reprojection_cost_function.cpp
 * @author Marc-Philip Ecker
 * @date 18.12.19
 */
#include <stereo_vio/processing/util.h>
#include "stereo_vio/ceres/reprojection_cost_function.h"

ReprojectionCostFunction::ReprojectionCostFunction(Eigen::Vector2d z, const Eigen::Matrix2d &covariance,
                                                   const Eigen::Quaterniond &q_body_cam, Eigen::Vector3d t_body_cam,
                                                   double fx, double fy, double cx, double cy) :
        z_(std::move(z)),
        q_body_cam_(q_body_cam),
        t_body_cam_(std::move(t_body_cam)),
        fx_(fx),
        fy_(fy),
        cx_(cx),
        cy_(cy)
{
    information_ = Eigen::LLT<Eigen::Matrix2d>(covariance).matrixL().transpose();
}

bool ReprojectionCostFunction::Evaluate(const double *const *parameters, double *residuals, double **jacobians) const
{
    // Map to ceres data structures
    const Eigen::Map<const Eigen::Quaterniond> q_world_body(parameters[0]);
    const Eigen::Map<const Eigen::Vector3d> t_world_body(parameters[1]);
    const Eigen::Map<const Eigen::Vector3d> l_world(parameters[2]);

    // Rotation matrices
    const Eigen::Matrix3d R_b_w(q_world_body.conjugate());
    const Eigen::Matrix3d R_c_b(q_body_cam_.conjugate());
    const Eigen::Matrix3d R_c_w = R_c_b * R_b_w;

    Eigen::Map<Eigen::Vector2d> r(residuals);

    // Calculate residual:

    // Transform landmark to camera frame
    const Eigen::Vector3d l_body = R_b_w * (l_world - t_world_body);
    const Eigen::Vector3d l_cam = R_c_b * (l_body - t_body_cam_);
    // Project landmark to image plane
    const Eigen::Vector2d p_hat = l_cam.block<2, 1>(0, 0) / l_cam.z();
    const Eigen::Vector2d z_hat(p_hat.x() * fx_ + cx_,
                                p_hat.y() * fy_ + cy_);
    // residual
    r = information_ * (z_hat - z_);

    // Calculate jacobians:

    // Jacobian of camera equation
    const double l_cam_z_inv = 1.0 / l_cam.z();
    const double l_cam_z_squared_inv = l_cam_z_inv * l_cam_z_inv;

    Eigen::Matrix<double, 2, 3> J_pi_lck;
    // Row 0:
    J_pi_lck(0, 0) = fx_ * l_cam_z_inv;
    J_pi_lck(0, 1) = 0;
    J_pi_lck(0, 2) = -(fx_ * l_cam.x()) * l_cam_z_squared_inv;
    // Row 1:
    J_pi_lck(1, 0) = 0;
    J_pi_lck(1, 1) = fy_ * l_cam_z_inv;
    J_pi_lck(1, 2) = -(fy_ * l_cam.y()) * l_cam_z_squared_inv;

    if (jacobians != nullptr)
    {
        if (jacobians[0] != nullptr)
        {
            // Jacobian of landmark in camera frame with respect to orientation
            Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>> J_h_qwbk(jacobians[0]);
            J_h_qwbk.block<2, 3>(0, 0) = J_pi_lck * R_c_b *  R_b_w * skew((l_world - t_world_body));;
            J_h_qwbk.block<2, 1>(0, 3).setZero();
        }
        if (jacobians[1] != nullptr)
        {
            Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> J_h_twbk(jacobians[1]);
            J_h_twbk = -J_pi_lck * R_c_w;
        }
        if (jacobians[2] != nullptr)
        {
            Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> J_h_lw(jacobians[2]);
            J_h_lw = J_pi_lck * R_c_w;
        }
    }

    return true;
}