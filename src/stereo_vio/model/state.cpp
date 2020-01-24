/**
 * @file state.cpp
 * @author Marc-Philip Ecker
 * @date 06.01.20
 */
#include "stereo_vio/model/state.h"

#define POSITION_START (0)
#define VELOCITY_START (3)
#define ORIENTATION_START (6)
#define BIAS_ACC_START (10)
#define BIAS_GYRO_START (13)

State::State() :
        Eigen::Matrix<double, STATE_SIZE, 1>(Eigen::Matrix<double, STATE_SIZE, 1>::Zero())
{
    *(data() + ORIENTATION_START + 3) = 1;
}

double *State::t_world_body_ptr()
{
    return data() + POSITION_START;
}

Eigen::Map<Eigen::Vector3d> State::t_world_body()
{
    return Eigen::Map<Eigen::Vector3d>(data() + POSITION_START);
}

const Eigen::Map<const Eigen::Vector3d> State::t_world_body() const
{
    return Eigen::Map<const Eigen::Vector3d>(data() + POSITION_START);
}

double *State::v_world_body_ptr()
{
    return data() + VELOCITY_START;
}

Eigen::Map<Eigen::Vector3d> State::v_world_body()
{
    return Eigen::Map<Eigen::Vector3d>(data() + VELOCITY_START);
}

const Eigen::Map<const Eigen::Vector3d> State::v_world_body() const
{
    return Eigen::Map<const Eigen::Vector3d>(data() + VELOCITY_START);
}

double *State::q_world_body_ptr()
{
    return data() + ORIENTATION_START;
}

Eigen::Map<Eigen::Quaterniond> State::q_world_body()
{
    return Eigen::Map<Eigen::Quaterniond>(data() + ORIENTATION_START);
}

const Eigen::Map<const Eigen::Quaterniond> State::q_world_body() const
{
    return Eigen::Map<const Eigen::Quaterniond>(data() + ORIENTATION_START);
}

double *State::bias_a_ptr()
{
    return data() + BIAS_ACC_START;
}

Eigen::Map<Eigen::Vector3d> State::bias_a()
{
    return Eigen::Map<Eigen::Vector3d>(data() + BIAS_ACC_START);
}

const Eigen::Map<const Eigen::Vector3d> State::bias_a() const
{
    return Eigen::Map<const Eigen::Vector3d>(data() + BIAS_ACC_START);
}

double *State::bias_w_ptr()
{
    return data() + BIAS_GYRO_START;
}

Eigen::Map<Eigen::Vector3d> State::bias_w()
{
    return Eigen::Map<Eigen::Vector3d>(data() + BIAS_GYRO_START);
}

const Eigen::Map<const Eigen::Vector3d> State::bias_w() const
{
    return Eigen::Map<const Eigen::Vector3d>(data() + BIAS_GYRO_START);
}

