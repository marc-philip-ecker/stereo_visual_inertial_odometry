/**
 * @file camera.cpp
 * @author Marc-Philip Ecker
 * @date 12.01.20
 */
#include "stereo_vio/sensors/camera.h"

Camera::Camera() :
        t_body_cam(0.0, 0.0, 0.0),
        q_body_cam(1.0, 0.0, 0.0, 0.0),
        camera_matrix(cv::Mat::eye(3, 3, CV_64F)),
        dist_coeffs(cv::Mat::zeros(4, 1, CV_64F))
{

}

Camera::Camera(Eigen::Vector3d t_body_cam_in, const Eigen::Quaterniond &q_body_cam_in,
               cv::Mat camera_matrix_in, cv::Mat dist_coeffs_in) :
        t_body_cam(std::move(t_body_cam_in)),
        q_body_cam(q_body_cam_in),
        camera_matrix(std::move(camera_matrix_in)),
        dist_coeffs(std::move(dist_coeffs_in))
{

}

Camera::Camera(const Camera &other) :
        t_body_cam(other.t_body_cam),
        q_body_cam(other.q_body_cam),
        camera_matrix(other.camera_matrix),
        dist_coeffs(other.dist_coeffs)
{

}

Eigen::Vector2d Camera::pixel_to_meter(const Eigen::Vector2d &z) const
{
    return {(z.x() - cx()) / fx(), (z.y() - cy()) / fy()};
}

Eigen::Vector2d Camera::meter_to_pixel(const Eigen::Vector2d &p) const
{
    return {p.x() * fx() + cx(), p.y() * fy() + cy()};
}

const double &Camera::fx() const
{
    return camera_matrix.at<double>(0, 0);
}

double &Camera::fx()
{
    return camera_matrix.at<double>(0, 0);
}

const double &Camera::fy() const
{
    return camera_matrix.at<double>(1, 1);
}

double &Camera::fy()
{
    return camera_matrix.at<double>(1, 1);
}

const double &Camera::cx() const
{
    return camera_matrix.at<double>(0, 2);
}

double &Camera::cx()
{
    return camera_matrix.at<double>(0, 2);
}

const double &Camera::cy() const
{
    return camera_matrix.at<double>(1, 2);
}

double &Camera::cy()
{
    return camera_matrix.at<double>(1, 2);
}