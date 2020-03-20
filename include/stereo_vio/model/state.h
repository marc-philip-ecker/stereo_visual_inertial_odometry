/**
 * @file state.h
 * @author Marc-Philip Ecker
 * @date 06.01.20
 */
#ifndef SRC_STATE_H
#define SRC_STATE_H

#include <Eigen/Dense>
#include <ros/ros.h>
#include <stereo_vio/model/model.h>

#define STATE_SIZE (16)

class State : public Eigen::Matrix<double, STATE_SIZE, 1>
{
public:
    ros::Time stamp;

    State();

    double *t_world_body_ptr();

    Eigen::Map<Eigen::Vector3d> t_world_body();

    const Eigen::Map<const Eigen::Vector3d> t_world_body() const;

    double *v_world_body_ptr();

    Eigen::Map<Eigen::Vector3d> v_world_body();

    const Eigen::Map<const Eigen::Vector3d> v_world_body() const;

    double *q_world_body_ptr();

    Eigen::Map<Eigen::Quaterniond> q_world_body();

    const Eigen::Map<const Eigen::Quaterniond> q_world_body() const;

    double *bias_a_ptr();

    Eigen::Map<Eigen::Vector3d> bias_a();

    const Eigen::Map<const Eigen::Vector3d> bias_a() const;

    double *bias_w_ptr();

    Eigen::Map<Eigen::Vector3d> bias_w();

    const Eigen::Map<const Eigen::Vector3d> bias_w() const;
};
#endif //SRC_STATE_H
