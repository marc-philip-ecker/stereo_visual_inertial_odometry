/**
 * @file measurement.h
 * @author Marc-Philip Ecker
 * @date 06.01.20
 */
#ifndef SRC_CAMERA_MEASUREMENT_H
#define SRC_CAMERA_MEASUREMENT_H

#include <Eigen/Dense>
#include <stereo_vio/sensors/stereo_camera.h>

enum CameraMeasurementState
{
    CM_INVALID,
    CM_INITIALIZED,
    CM_TRACKED
};

class CameraMeasurement
{
public:

    Eigen::Vector2d z0;

    Eigen::Vector2d z1;

    Eigen::Vector2d z0_dist;

    Eigen::Vector2d z1_dist;

    bool is_keyframe;

    int landmark_id;

    CameraMeasurement();

    Eigen::Vector3d stereo_triangulate(const StereoCamera &stereo_camera) const;

    bool check_epipolar_constraint(const StereoCamera &stereo_camera, double threshold);

    bool is_invalid() const;

    bool is_invalid();

    bool is_initialized() const;

    bool is_initialized();

    bool is_tracked() const;

    bool is_tracked();

    void set_invalid();

    void set_initialized();

    void set_tracked();

private:
    CameraMeasurementState state_;
};

#endif //SRC_CAMERA_MEASUREMENT_H
