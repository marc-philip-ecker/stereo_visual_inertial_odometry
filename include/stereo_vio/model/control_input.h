/**
 * @file control_input.h
 * @author Marc-Philip Ecker
 * @date 09.01.20
 */
#ifndef SRC_CONTROL_INPUT_H
#define SRC_CONTROL_INPUT_H

#include <Eigen/Dense>
#include <vector>
#include <ros/ros.h>

#include <stereo_vio/model/model.h>


#define CONTROL_INPUT_SIZE (10)

class ControlInput : public Eigen::Matrix<double, CONTROL_INPUT_SIZE, 1>
{
public:
    std::vector<ros::Time> stamps;

    std::vector<ImuMeasurement> measurements;

    Eigen::Matrix<double, CONTROL_INPUT_SIZE + 5, CONTROL_INPUT_SIZE + 5> F;

    Eigen::Matrix<double, CONTROL_INPUT_SIZE + 5, CONTROL_INPUT_SIZE + 5> Q;

    Eigen::Vector3d bias_a;

    Eigen::Vector3d bias_w;

    ControlInput();

    Eigen::Map<Eigen::Vector3d> alpha();

    Eigen::Map<const Eigen::Vector3d> alpha() const;

    Eigen::Map<Eigen::Vector3d> beta();

    Eigen::Map<const Eigen::Vector3d> beta() const;

    Eigen::Map<Eigen::Quaterniond> gamma();

    Eigen::Map<const Eigen::Quaterniond> gamma() const;

private:

};
#endif //SRC_CONTROL_INPUT_H
