/**
 * @file imu_cost_function.cpp
 * @author Marc-Philip Ecker
 * @date 12.01.20
 */
#include "stereo_vio/ceres/imu_cost_function.h"

#include <stereo_vio/processing/util.h>

ImuCostFunction::ImuCostFunction(ControlInput u)
        : dt_((u.stamps.back() - u.stamps.front()).toSec())
{
    u_ = u;
    g_ = -9.81 * Eigen::Vector3d::UnitZ();
    information_sqrt_ = Eigen::LLT<Eigen::Matrix<double, 15, 15>>(u_.Q.inverse()).matrixL().transpose();
}

bool ImuCostFunction::Evaluate(const double *const *parameters, double *residuals, double **jacobians) const
{
    const Eigen::Map<const Eigen::Vector3d> t_w_bk(parameters[0]);
    const Eigen::Map<const Eigen::Vector3d> v_w_bk(parameters[1]);
    const Eigen::Map<const Eigen::Quaterniond> q_w_bk(parameters[2]);
    const Eigen::Map<const Eigen::Vector3d> b_ak(parameters[3]);
    const Eigen::Map<const Eigen::Vector3d> b_wk(parameters[4]);

    const Eigen::Map<const Eigen::Vector3d> t_w_bkp1(parameters[5]);
    const Eigen::Map<const Eigen::Vector3d> v_w_bkp1(parameters[6]);
    const Eigen::Map<const Eigen::Quaterniond> q_w_bkp1(parameters[7]);
    const Eigen::Map<const Eigen::Vector3d> b_akp1(parameters[8]);
    const Eigen::Map<const Eigen::Vector3d> b_wkp1(parameters[9]);

    Eigen::Map<Eigen::Vector3d> r_t(residuals);
    Eigen::Map<Eigen::Vector3d> r_v(residuals + 3);
    Eigen::Map<Eigen::Vector3d> r_q(residuals + 6);
    Eigen::Map<Eigen::Vector3d> r_ba(residuals + 9);
    Eigen::Map<Eigen::Vector3d> r_bw(residuals + 12);
    Eigen::Map<Eigen::Matrix<double, 15, 1>> r(residuals);

    const Eigen::Matrix3d R_bk_w(q_w_bk.conjugate());

    // Jacobians for correction
    const Eigen::Matrix3d &dalpha_dba = u_.F.block<3, 3>(0, 9);
    const Eigen::Matrix3d &dalpha_dbw = u_.F.block<3, 3>(0, 12);

    const Eigen::Matrix3d &dbeta_dba = u_.F.block<3, 3>(3, 9);
    const Eigen::Matrix3d &dbeta_dbw = u_.F.block<3, 3>(3, 12);

    const Eigen::Matrix3d &dgamma_dbw = u_.F.block<3, 3>(6, 12);

    const Eigen::Vector3d dtheta = dgamma_dbw * (b_wk - u_.bias_w);

    // Correction
    const Eigen::Vector3d alpha_corrected = u_.alpha() + dalpha_dba * (b_ak - u_.bias_a)
                                            + dalpha_dbw * (b_wk - u_.bias_w);
    const Eigen::Vector3d beta_corrected = u_.beta() + dbeta_dba * (b_ak - u_.bias_a)
                                           + dbeta_dbw * (b_wk - u_.bias_w);
    const Eigen::Quaterniond gamma_corrected = u_.gamma() * Eigen::Quaterniond(
            Eigen::AngleAxisd(dtheta.norm(), dtheta.normalized()));

    r_t = R_bk_w * (t_w_bkp1 - t_w_bk - 0.5 * g_ * dt_ * dt_ - v_w_bk * dt_) - alpha_corrected;
    r_v = R_bk_w * (v_w_bkp1 - g_ * dt_ - v_w_bk) - beta_corrected;
    r_q = 2 * (q_w_bkp1.conjugate() * q_w_bk * gamma_corrected).vec();
    r_ba = b_akp1 - b_ak;
    r_bw = b_wkp1 - b_wk;

    r = information_sqrt_ * r;

    if (jacobians)
    {
        // t_w_bk
        if (jacobians[0])
        {
            Eigen::Map<Eigen::Matrix<double, 15, 3, Eigen::RowMajor>> J_f_twbk(jacobians[0]);
            J_f_twbk.setZero();
            J_f_twbk.block<3, 3>(0, 0) = -R_bk_w;

            J_f_twbk.block<15, 3>(0, 0) = information_sqrt_ * J_f_twbk.block<15, 3>(0, 0);
        }
        // v_w_bk
        if (jacobians[1])
        {
            Eigen::Map<Eigen::Matrix<double, 15, 3, Eigen::RowMajor>> J_f_vwbk(jacobians[1]);
            J_f_vwbk.setZero();
            J_f_vwbk.block<3, 3>(0, 0) = -R_bk_w * dt_;
            J_f_vwbk.block<3, 3>(3, 0) = -R_bk_w;

            J_f_vwbk.block<15, 3>(0, 0) = information_sqrt_ * J_f_vwbk.block<15, 3>(0, 0);
        }
        // q_w_bk
        if (jacobians[2])
        {
            const Eigen::Matrix4d product =
                    quaternion_left(q_w_bkp1.conjugate()) * quaternion_right(q_w_bk * gamma_corrected);

            Eigen::Map<Eigen::Matrix<double, 15, 4, Eigen::RowMajor>> J_f_qwbk(jacobians[2]);
            J_f_qwbk.setZero();
            J_f_qwbk.block<3, 3>(0, 0) = R_bk_w * skew((t_w_bkp1 - t_w_bk - 0.5 * g_ * dt_ * dt_ - v_w_bk * dt_));
            J_f_qwbk.block<3, 3>(3, 0) = R_bk_w * skew((v_w_bkp1 - g_ * dt_ - v_w_bk));
            J_f_qwbk.block<3, 3>(6, 0) = product.block<3, 3>(0, 0);

            J_f_qwbk.block<15, 3>(0, 0) = information_sqrt_ * J_f_qwbk.block<15, 3>(0, 0);
        }
        // b_ak
        if (jacobians[3])
        {
            Eigen::Map<Eigen::Matrix<double, 15, 3, Eigen::RowMajor>> J_f_bak(jacobians[3]);
            J_f_bak.setZero();
            J_f_bak.block<3, 3>(0, 0) = -dalpha_dba;
            J_f_bak.block<3, 3>(3, 0) = -dbeta_dba;
            J_f_bak.block<3, 3>(9, 0) = -Eigen::Matrix3d::Identity();

            J_f_bak.block<15, 3>(0, 0) = information_sqrt_ * J_f_bak.block<15, 3>(0, 0);
        }
        // b_wk
        if (jacobians[4])
        {
            const Eigen::Matrix4d product =
                    quaternion_left(q_w_bkp1.conjugate() * q_w_bk) * quaternion_right(gamma_corrected);

            Eigen::Map<Eigen::Matrix<double, 15, 3, Eigen::RowMajor>> J_f_bwk(jacobians[4]);
            J_f_bwk.setZero();
            J_f_bwk.block<3, 3>(0, 0) = -dalpha_dbw;
            J_f_bwk.block<3, 3>(3, 0) = -dbeta_dbw;
            J_f_bwk.block<3, 3>(6, 0) = product.block<3, 3>(0, 0) * dgamma_dbw;
            J_f_bwk.block<3, 3>(12, 0) = -Eigen::Matrix3d::Identity();

            J_f_bwk.block<15, 3>(0, 0) = information_sqrt_ * J_f_bwk.block<15, 3>(0, 0);
        }
        // t_w_bkp1
        if (jacobians[5])
        {
            Eigen::Map<Eigen::Matrix<double, 15, 3, Eigen::RowMajor>> J_f_twbkp1(jacobians[5]);
            J_f_twbkp1.setZero();
            J_f_twbkp1.block<3, 3>(0, 0) = R_bk_w;

            J_f_twbkp1.block<15, 3>(0, 0) = information_sqrt_ * J_f_twbkp1.block<15, 3>(0, 0);
        }
        // v_w_bkp1
        if (jacobians[6])
        {
            Eigen::Map<Eigen::Matrix<double, 15, 3, Eigen::RowMajor>> J_f_vwbkp1(jacobians[6]);
            J_f_vwbkp1.setZero();
            J_f_vwbkp1.block<3, 3>(3, 0) = R_bk_w;

            J_f_vwbkp1.block<15, 3>(0, 0) = information_sqrt_ * J_f_vwbkp1.block<15, 3>(0, 0);
        }
        // q_w_bkp1
        if (jacobians[7])
        {
            const Eigen::Matrix4d product = quaternion_left(q_w_bkp1.conjugate())
                                            * quaternion_right(q_w_bk * gamma_corrected);

            Eigen::Map<Eigen::Matrix<double, 15, 4, Eigen::RowMajor>> J_f_qwbkp1(jacobians[7]);
            J_f_qwbkp1.setZero();
            J_f_qwbkp1.block<3, 3>(6, 0) = -product.block<3, 3>(0, 0);

            J_f_qwbkp1.block<15, 3>(0, 0) = information_sqrt_ * J_f_qwbkp1.block<15, 3>(0, 0);
        }
        // b_akp1
        if (jacobians[8])
        {
            Eigen::Map<Eigen::Matrix<double, 15, 3, Eigen::RowMajor>> J_f_bakp1(jacobians[8]);
            J_f_bakp1.setZero();
            J_f_bakp1.block<3, 3>(9, 0) = Eigen::Matrix3d::Identity();

            J_f_bakp1.block<15, 3>(0, 0) = information_sqrt_ * J_f_bakp1.block<15, 3>(0, 0);
        }
        // b_wkp1
        if (jacobians[9])
        {
            Eigen::Map<Eigen::Matrix<double, 15, 3, Eigen::RowMajor>> J_f_bwkp1(jacobians[9]);
            J_f_bwkp1.setZero();
            J_f_bwkp1.block<3, 3>(12, 0) = Eigen::Matrix3d::Identity();

            J_f_bwkp1.block<15, 3>(0, 0) = information_sqrt_ * J_f_bwkp1.block<15, 3>(0, 0);
        }
    }

    return true;
}