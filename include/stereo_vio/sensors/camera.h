/**
 * @file camera.h
 * @author Marc-Philip Ecker
 * @date 06.01.20
 */
#ifndef SRC_CAMERA_H
#define SRC_CAMERA_H

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

class Camera
{
public:
    Eigen::Vector3d t_body_cam;
    Eigen::Quaterniond q_body_cam;
    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;

    Camera();

    Camera(Eigen::Vector3d t_body_cam_in, const Eigen::Quaterniond &q_body_cam_in,
            cv::Mat camera_matrix_in, cv::Mat dist_coeffs_in);

    Camera(const Camera &other);

    Eigen::Vector2d pixel_to_meter(const Eigen::Vector2d &z) const;

    Eigen::Vector2d meter_to_pixel(const Eigen::Vector2d &p) const;

    const double &fx() const;

    double &fx();

    const double &fy() const;

    double &fy();

    const double &cx() const;

    double &cx();

    const double &cy() const;

    double &cy();

private:
};
#endif //SRC_CAMERA_H
