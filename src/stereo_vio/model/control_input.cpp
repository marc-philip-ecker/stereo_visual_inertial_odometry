/**
 * @file control_input.cpp
 * @author Marc-Philip Ecker
 * @date 09.01.20
 */
#include "stereo_vio/model/control_input.h"

#define ALPHA_POSITION_START (0)
#define BETA_POSITION_START (3)
#define GAMMA_POSITION_START (6)

ControlInput::ControlInput() :
        Eigen::Matrix<double, CONTROL_INPUT_SIZE, 1>(Eigen::Matrix<double, CONTROL_INPUT_SIZE, 1>::Zero()),
        stamps(0),
        measurements(0),
        F(Eigen::Matrix<double, CONTROL_INPUT_SIZE + 5, CONTROL_INPUT_SIZE + 5>::Identity()),
        Q(Eigen::Matrix<double, CONTROL_INPUT_SIZE + 5, CONTROL_INPUT_SIZE + 5>::Identity()),
        bias_a(0.0, 0.0, 0.0),
        bias_w(0.0, 0.0, 0.0)
{
    *(data() + GAMMA_POSITION_START + 3) = 1.0;
}

Eigen::Map<Eigen::Vector3d> ControlInput::alpha()
{
    return Eigen::Map<Eigen::Vector3d>(data() + ALPHA_POSITION_START);
}

Eigen::Map<const Eigen::Vector3d> ControlInput::alpha() const
{
    return Eigen::Map<const Eigen::Vector3d>(data() + ALPHA_POSITION_START);
}

Eigen::Map<Eigen::Vector3d> ControlInput::beta()
{
    return Eigen::Map<Eigen::Vector3d>(data() + BETA_POSITION_START);
}

Eigen::Map<const Eigen::Vector3d> ControlInput::beta() const
{
    return Eigen::Map<const Eigen::Vector3d>(data() + BETA_POSITION_START);
}

Eigen::Map<Eigen::Quaterniond> ControlInput::gamma()
{
    return Eigen::Map<Eigen::Quaterniond>(data() + GAMMA_POSITION_START);
}

Eigen::Map<const Eigen::Quaterniond> ControlInput::gamma() const
{
    return Eigen::Map<const Eigen::Quaterniond>(data() + GAMMA_POSITION_START);
}