/**
 * @file pre_integrator.cpp
 * @author Marc-Philip Ecker
 * @date 21.12.19
 */
#include "stereo_vio/processing/pre_integrator.h"

PreIntegrator::PreIntegrator(double sigma_a, double sigma_ba, double sigma_w, double sigma_bw) :
        sigma_a_(sigma_a),
        sigma_w_(sigma_w),
        sigma_ba_(sigma_ba),
        sigma_bw_(sigma_bw),
        I33_(Eigen::Matrix3d::Identity())
{
    F_noise_.setZero();
    F_noise_.block<3, 3>(3, 0).setIdentity();
    F_noise_.block<3, 3>(6, 3).setIdentity();
    F_noise_.block<3, 3>(9, 6).setIdentity();
    F_noise_.block<3, 3>(12, 9).setIdentity();
}

void PreIntegrator::pre_integrate()
{
    F_minimal_discrete_.setIdentity();

    u_.F.setIdentity();
    u_.Q.setIdentity();
    //u_.Q.setZero();
    u_.Q *= 1e-6;
    //u_.Q.block<6, 6>(9, 9).setIdentity();
    u_.alpha().setZero();
    u_.beta().setZero();
    u_.gamma().setIdentity();
    u_.bias_a = x_.bias_a();
    u_.bias_w = x_.bias_w();

    if (u_.stamps.empty())
        return;

    for (size_t i = 0; i < (u_.stamps.size() - 1); ++i)
    {
        // Calculate delta time
        const double t_i0 = u_.stamps[i].toSec();
        const double t_i1 = u_.stamps[i + 1].toSec();
        const double dt = t_i1 - t_i0;

        update_control_jacobian(u_.measurements[i], dt);
        update_control_covariance(dt);
        update_control(u_.measurements[i], dt);
    }
}

void PreIntegrator::update_control(const ImuMeasurement &measurement, double dt)
{
    const Eigen::Map<const Eigen::Vector3d> a(measurement.data() + ACC_START);
    const Eigen::Map<const Eigen::Vector3d> w(measurement.data() + ANG_START);

    // Integrate position
    u_.alpha() += u_.beta() * dt + 0.5 * (u_.gamma() * (a - x_.bias_a())) * dt * dt;

    // Integrate velocity
    u_.beta() += u_.gamma() * (a - x_.bias_a()) * dt;

    // Integrate angular velocity
    const Eigen::Vector3d dtheta_vec = (w - x_.bias_w()) * dt;
    const Eigen::AngleAxisd dtheta_aa(dtheta_vec.norm(), dtheta_vec.normalized());
    const Eigen::Quaterniond dq(dtheta_aa);
    u_.gamma() = u_.gamma() * dq;
}

void PreIntegrator::update_control_jacobian(const ImuMeasurement &measurement, double dt)
{
    // Measurements
    const Eigen::Map<const Eigen::Vector3d> a(measurement.data());

    // Rotation matrix
    const Eigen::Matrix3d R_gamma(u_.gamma());
    const Eigen::Matrix3d minusR_gamma_dt = -R_gamma * dt;
    // Temporary calculations
    const Eigen::Vector3d a_transformed = R_gamma * (a - x_.bias_a());
    const Eigen::Matrix3d skew = (const Eigen::Matrix3d)
            (Eigen::Matrix3d() << 0, -a_transformed.z(), a_transformed.y(),
                    a_transformed.z(), 0, -a_transformed.x(),
                    -a_transformed.y(), a_transformed.x(), 0).finished();

    // Update Jacobian
    F_minimal_discrete_.block<3, 3>(0, 3) = I33_ * dt;
    F_minimal_discrete_.block<3, 3>(0, 6) = -0.5 * skew * dt * dt;
    F_minimal_discrete_.block<3, 3>(0, 9) = 0.5 * minusR_gamma_dt * dt;
    F_minimal_discrete_.block<3, 3>(0, 12) = 0.5 * minusR_gamma_dt * dt;
    F_minimal_discrete_.block<3, 3>(3, 6) = -skew * dt;
    F_minimal_discrete_.block<3, 3>(3, 9) = minusR_gamma_dt;
    F_minimal_discrete_.block<3, 3>(6, 12) = minusR_gamma_dt;

    F_noise_.block<3, 3>(0, 0) = 0.5 * minusR_gamma_dt * dt;
    F_noise_.block<3, 3>(3, 0) = minusR_gamma_dt;
    F_noise_.block<3, 3>(6, 3) = minusR_gamma_dt;

    u_.F = F_minimal_discrete_ * u_.F;
}

void PreIntegrator::update_control_covariance(double dt)
{
    Eigen::Matrix<double, 12, 12> V = Eigen::Matrix<double, 12, 12>::Zero();
    V.block<3, 3>(0, 0) = sigma_a_ * sigma_a_ * I33_;
    V.block<3, 3>(3, 3) = sigma_w_ * sigma_w_ * I33_;
    V.block<3, 3>(6, 6) = sigma_ba_ * sigma_ba_ * dt * I33_;
    V.block<3, 3>(9, 9) = sigma_bw_ * sigma_bw_ * dt * I33_;

    u_.Q = F_minimal_discrete_ * u_.Q * F_minimal_discrete_.transpose() + F_noise_ * V * F_noise_.transpose();

    u_.Q = (u_.Q + u_.Q.transpose()) * 0.5;
}

State &PreIntegrator::x()
{
    return x_;
}

ControlInput &PreIntegrator::u()
{
    return u_;
}
