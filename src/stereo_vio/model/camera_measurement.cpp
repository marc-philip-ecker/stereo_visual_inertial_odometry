/**
 * @file measurement.cpp
 * @author Marc-Philip Ecker
 * @date 06.01.20
 */

#include <stereo_vio/model/camera_measurement.h>

CameraMeasurement::CameraMeasurement() :
        z0(0.0, 0.0),
        z1(0.0, 0.0),
        z0_dist(0.0, 0.0),
        z1_dist(0.0, 0.0),
        is_keyframe(false),
        landmark_id(-1),
        state_(CM_INVALID)
{

}

Eigen::Vector3d CameraMeasurement::stereo_triangulate(const StereoCamera &stereo_camera) const
{
    // Convert pixel to meters
    const Eigen::Vector2d p0 = stereo_camera.cam0.pixel_to_meter(z0);
    const Eigen::Vector2d p1 = stereo_camera.cam1.pixel_to_meter(z1);

    // Create homogeneous coordinates
    const Eigen::Vector3d p0_tilde(p0.x(), p0.y(), 1.0);
    const Eigen::Vector3d p1_tilde(p1.x(), p1.y(), 1.0);

    // Build least squares problem
    const Eigen::Vector3d m0 = stereo_camera.q_cam0_cam1 * p1_tilde;

    const Eigen::Vector2d H(p0_tilde.x() * m0.z() - m0.x(),
                            p0_tilde.y() * m0.z() - m0.y());
    const Eigen::Vector2d y(stereo_camera.t_cam0_cam1.x() - p0_tilde.x() * stereo_camera.t_cam0_cam1.z(),
                            stereo_camera.t_cam0_cam1.y() - p0_tilde.y() * stereo_camera.t_cam0_cam1.z());

    // Solve least squares problem
    const Eigen::RowVector2d HT = H.transpose();
    const double HTH = HT * H;
    const double HTy = HT * y;

    const double lambda1 = HTy / HTH;

    // Build landmark in camera frame
    const Eigen::Vector3d l_c1 = p1_tilde * lambda1;

    // Transform to body frame
    const Eigen::Vector3d l_b = stereo_camera.cam1.q_body_cam * l_c1 + stereo_camera.cam1.t_body_cam;

    return l_b;
}

bool CameraMeasurement::check_epipolar_constraint(const StereoCamera &stereo_camera, double threshold)
{
    // Convert pixel to meters
    const Eigen::Vector2d p0 = stereo_camera.cam0.pixel_to_meter(z0);
    const Eigen::Vector2d p1 = stereo_camera.cam1.pixel_to_meter(z1);

    // Create homogeneous coordinates
    const Eigen::Vector3d p0_tilde(p0.x(), p0.y(), 1.0);
    const Eigen::Vector3d p1_tilde(p1.x(), p1.y(), 1.0);

    // Calculate epipolar value, i.e. p0_tilde^T * E * p1_tilde
    double epipolar_value = p0_tilde.dot(stereo_camera.t_cam0_cam1.cross(stereo_camera.q_cam0_cam1 * p1_tilde));

    // Return true if epipolar value < threshold and false otherwise
    return abs(epipolar_value) < threshold;
}

bool CameraMeasurement::is_invalid() const
{
    return state_ == CM_INVALID;
}

bool CameraMeasurement::is_invalid()
{
    return state_ == CM_INVALID;
}

bool CameraMeasurement::is_initialized() const
{
    return state_ == CM_INITIALIZED;
}

bool CameraMeasurement::is_initialized()
{
    return state_ == CM_INITIALIZED;
}

bool CameraMeasurement::is_tracked() const
{
    return state_ == CM_TRACKED;
}

bool CameraMeasurement::is_tracked()
{
    return state_ == CM_TRACKED;
}

void CameraMeasurement::set_invalid()
{
    state_ = CM_INVALID;
}

void CameraMeasurement::set_initialized()
{
    state_ = CM_INITIALIZED;
}

void CameraMeasurement::set_tracked()
{
    state_ = CM_TRACKED;
}