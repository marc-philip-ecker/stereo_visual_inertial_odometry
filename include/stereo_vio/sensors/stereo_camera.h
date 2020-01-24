/**
 * @file stereo_camera.h
 * @author Marc-Philip Ecker
 * @date 12.01.20
 */
#ifndef SRC_STEREO_CAMERA_H
#define SRC_STEREO_CAMERA_H

#include <stereo_vio/sensors/camera.h>

class StereoCamera
{
public:
    Eigen::Vector3d t_cam0_cam1;
    Eigen::Quaterniond q_cam0_cam1;

    Camera cam0, cam1;

    StereoCamera(const Camera &cam0_in, const Camera &cam1_in);

    StereoCamera(const StereoCamera &other);
};
#endif //SRC_STEREO_CAMERA_H
