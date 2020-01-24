/**
 * @file pre_integrator.h
 * @author Marc-Philip Ecker
 * @date 08.01.20
 */
#ifndef SRC_PRE_INTEGRATOR_H
#define SRC_PRE_INTEGRATOR_H

#include <stereo_vio/model/model.h>
#include <stereo_vio/model/state.h>
#include <stereo_vio/model/control_input.h>

#include <vector>
#include <ros/ros.h>

class PreIntegrator
{
public:
    PreIntegrator(double sigma_a, double sigma_ba, double sigma_w, double sigma_bw);

    void pre_integrate();

    State &x();

    ControlInput &u();

private:
    State x_;

    ControlInput u_;

    Eigen::Matrix<double, CONTROL_INPUT_SIZE + 5, CONTROL_INPUT_SIZE + 5> F_minimal_discrete_;

    Eigen::Matrix<double, CONTROL_INPUT_SIZE + 5, 12> F_noise_;

    double sigma_a_, sigma_w_, sigma_ba_, sigma_bw_;

    const Eigen::Matrix3d I33_;

    void update_control(const ImuMeasurement &measurement, double dt);

    void update_control_jacobian(const ImuMeasurement &measurement, double dt);

    void update_control_covariance(double dt);
};

#endif //SRC_PRE_INTEGRATOR_H
