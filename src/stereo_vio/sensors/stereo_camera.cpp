/**
 * @file stereo_camera.cpp
 * @author Marc-Philip Ecker
 * @date 12.01.20
 */
#include "stereo_vio/sensors/stereo_camera.h"

StereoCamera::StereoCamera(const Camera &cam0_in, const Camera &cam1_in) :
        cam0(cam0_in),
        cam1(cam1_in),
        t_cam0_cam1(cam0_in.q_body_cam.conjugate() * (cam1_in.t_body_cam - cam0_in.t_body_cam)),
        q_cam0_cam1(cam0_in.q_body_cam.conjugate() * cam1_in.q_body_cam)
{

}

StereoCamera::StereoCamera(const StereoCamera &other) :
        StereoCamera(other.cam0, other.cam1)
{

}