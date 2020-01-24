/**
 * @file feature_tracker.h
 * @author Marc-Philip Ecker
 * @date 21.12.19
 */
#ifndef SRC_FEATURETRACKER_H
#define SRC_FEATURETRACKER_H

#include <opencv2/opencv.hpp>
#include <stereo_vio/model/camera_measurement.h>
#include <stereo_vio/sensors/stereo_camera.h>

class FeatureTracker
{
public:
    FeatureTracker(const StereoCamera &stereo_camera, size_t points_per_image, size_t point_displacement,
                   double epipolar_outlier_threshold);

    void process_image_pair(const cv::Mat &image0, const cv::Mat &image1);

    std::vector<CameraMeasurement> &curr_measurements();

private:
    std::vector<CameraMeasurement> prev_measurements_;
    std::vector<CameraMeasurement> curr_measurements_;

    StereoCamera stereo_camera_;

    cv::Mat prev_image0_, prev_image1_;

    size_t points_per_image_, point_displacement_;

    double epipolar_outlier_threshold_;

    cv::TermCriteria corner_sub_pix_crit_;

    bool enable_visualization_;

    void update_points(const cv::Mat &image0,
                       const cv::Mat &image1,
                       size_t points_required,
                       const cv::InputArray &mask = cv::noArray());

    void visualize();
};

#endif //SRC_FEATURETRACKER_H
