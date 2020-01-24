/**
 * @file feature_tracker.cpp
 * @author Marc-Philip Ecker
 * @date 21.12.19
 */
#include "stereo_vio/processing/feature_tracker.h"

FeatureTracker::FeatureTracker(const StereoCamera &stereo_camera, size_t points_per_image, size_t point_displacement,
                               double epipolar_outlier_threshold) :
        stereo_camera_(stereo_camera),
        prev_measurements_(points_per_image),
        curr_measurements_(points_per_image),
        prev_image0_(0, 0, CV_8U),
        prev_image1_(0, 0, CV_8U),
        points_per_image_(points_per_image),
        point_displacement_(point_displacement),
        epipolar_outlier_threshold_(epipolar_outlier_threshold),
        enable_visualization_(false),
        corner_sub_pix_crit_(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 20, 0.03)
{

}

void FeatureTracker::process_image_pair(const cv::Mat &image0, const cv::Mat &image1)
{
    // Update previous points
    prev_measurements_ = std::vector<CameraMeasurement>(curr_measurements_);

    std::vector<cv::Point2f> prev_valid_points0(0), prev_valid_points1(0);

    for (const auto &measurement : prev_measurements_)
    {
        if (measurement.is_invalid())
            continue;

        const cv::Point2f point0(measurement.z0_dist.x(), measurement.z0_dist.y());
        const cv::Point2f point1(measurement.z1_dist.x(), measurement.z1_dist.y());

        prev_valid_points0.push_back(point0);
        prev_valid_points1.push_back(point1);
    }

    if (prev_valid_points0.empty())
    {
        // No previous points available -> generate new ones
        update_points(image0, image1, points_per_image_);
    }
    else
    {
        // Previous points available -> track and generate new ones afterwards
        // Temporal storage
        std::vector<cv::Point2f> new_points0_dist(0), new_points1_dist(0);
        std::vector<cv::Point2f> new_points0(0), new_points1(0);

        // Track features 0
        std::vector<unsigned char> status0;
        std::vector<float> error0;
        cv::calcOpticalFlowPyrLK(prev_image0_, image0, prev_valid_points0, new_points0_dist, status0, error0);
        //cv::cornerSubPix(image0, new_points0_dist, cv::Size(10, 10), cv::Size(-1, -1), corner_sub_pix_crit_);
        cv::undistortPoints(new_points0_dist, new_points0, stereo_camera_.cam0.camera_matrix,
                            stereo_camera_.cam0.dist_coeffs, cv::noArray(), stereo_camera_.cam0.camera_matrix);

        // Track features 1
        std::vector<unsigned char> status1;
        std::vector<float> error1;
        cv::calcOpticalFlowPyrLK(prev_image1_, image1, prev_valid_points1, new_points1_dist, status1, error1);
        //cv::cornerSubPix(image1, new_points1_dist, cv::Size(10, 10), cv::Size(-1, -1), corner_sub_pix_crit_);
        cv::undistortPoints(new_points1_dist, new_points1, stereo_camera_.cam1.camera_matrix,
                            stereo_camera_.cam1.dist_coeffs, cv::noArray(), stereo_camera_.cam1.camera_matrix);

        // Mask used to mark areas of valid measurements. This is used to force detector to get feature with good
        // displacement.
        size_t points_required = 0;
        cv::Mat mask(image0.rows, image0.cols, CV_8U, cv::Scalar(255));
        for (size_t i = 0; i < prev_measurements_.size(); ++i)
        {
            if (prev_measurements_[i].is_invalid() || status0.empty())
            {
                points_required++;
                prev_measurements_[i].set_invalid();

                continue;
            }

            if (status0.front() == 1
                && status1.front() == 1
                && mask.at<uint8_t>(new_points0_dist.front().y, new_points0_dist.front().x) != 0)
            {
                curr_measurements_[i].z0_dist = Eigen::Vector2d(new_points0_dist.front().x, new_points0_dist.front().y);
                curr_measurements_[i].z1_dist = Eigen::Vector2d(new_points1_dist.front().x, new_points1_dist.front().y);

                curr_measurements_[i].z0 = Eigen::Vector2d(new_points0.front().x, new_points0.front().y);
                curr_measurements_[i].z1 = Eigen::Vector2d(new_points1.front().x, new_points1.front().y);

                if (curr_measurements_[i].check_epipolar_constraint(stereo_camera_, epipolar_outlier_threshold_))
                {
                    curr_measurements_[i].set_tracked();
                }
                else
                {
                    curr_measurements_[i].set_invalid();
                    points_required++;

                    new_points0_dist.erase(new_points0_dist.begin());
                    new_points1_dist.erase(new_points1_dist.begin());

                    new_points0.erase(new_points0.begin());
                    new_points1.erase(new_points1.begin());

                    status0.erase(status0.begin());
                    status1.erase(status1.begin());

                    continue;
                }

                // Update mask that marks valid point areas
                cv::Point2i coordinate_lower_bound(round(curr_measurements_[i].z0_dist.x() - point_displacement_),
                                                   round(curr_measurements_[i].z0_dist.y() - point_displacement_));
                cv::Point2i coordinate_upper_bound(round(curr_measurements_[i].z0_dist.x() + point_displacement_),
                                                   round(curr_measurements_[i].z0_dist.y() + point_displacement_));

                // Assert image bounds
                coordinate_lower_bound.x = std::max(coordinate_lower_bound.x, 0);
                coordinate_lower_bound.y = std::max(coordinate_lower_bound.y, 0);

                coordinate_upper_bound.x = std::min(coordinate_upper_bound.x, image0.cols - 1);
                coordinate_upper_bound.y = std::min(coordinate_upper_bound.y, image0.rows - 1);

                // Draw rectangle around valid point
                cv::rectangle(mask, coordinate_lower_bound, coordinate_upper_bound, cv::Scalar(0), cv::FILLED);
            }
            else
            {
                curr_measurements_[i].set_invalid();
                points_required++;
            }

            new_points0_dist.erase(new_points0_dist.begin());
            new_points1_dist.erase(new_points1_dist.begin());

            new_points0.erase(new_points0.begin());
            new_points1.erase(new_points1.begin());

            status0.erase(status0.begin());
            status1.erase(status1.begin());
        }

        // Generate new points
        update_points(image0, image1, points_required, mask);
    }

    prev_image0_ = image0;
    prev_image1_ = image1;

    if (enable_visualization_)
    {
        visualize();
    }
}

std::vector<CameraMeasurement> &FeatureTracker::curr_measurements()
{
    return curr_measurements_;
}

void FeatureTracker::update_points(const cv::Mat &image0,
                                   const cv::Mat &image1,
                                   size_t points_required,
                                   const cv::InputArray &mask)
{
    if (points_required == 0)
        return;

    // Temporal storage
    std::vector<cv::Point2f> new_points0_dist(0), new_points1_dist(0);
    std::vector<cv::Point2f> new_points0(0), new_points1(0);

    // Generate new features
    cv::goodFeaturesToTrack(image0, new_points0_dist, points_required, 0.001, point_displacement_, mask);
    //cv::cornerSubPix(image0, new_points0_dist, cv::Size(10, 10), cv::Size(-1, -1), corner_sub_pix_crit_);
    cv::undistortPoints(new_points0_dist, new_points0, stereo_camera_.cam0.camera_matrix,
                        stereo_camera_.cam0.dist_coeffs, cv::noArray(), stereo_camera_.cam0.camera_matrix);

    // Track to image1
    std::vector<unsigned char> status;
    std::vector<float> error;
    cv::calcOpticalFlowPyrLK(image0, image1, new_points0_dist, new_points1_dist, status, error);
    //cv::cornerSubPix(image1, new_points1_dist, cv::Size(10, 10), cv::Size(-1, -1), corner_sub_pix_crit_);
    cv::undistortPoints(new_points1_dist, new_points1, stereo_camera_.cam1.camera_matrix,
                        stereo_camera_.cam1.dist_coeffs, cv::noArray(), stereo_camera_.cam1.camera_matrix);

    for (auto &measurement : curr_measurements_)
    {
        if (status.empty())
            break;

        if (!measurement.is_invalid())
            continue;

        if (status.front() == 1)
        {
            measurement.z0_dist = Eigen::Vector2d(new_points0_dist.front().x, new_points0_dist.front().y);
            measurement.z1_dist = Eigen::Vector2d(new_points1_dist.front().x, new_points1_dist.front().y);

            measurement.z0 = Eigen::Vector2d(new_points0.front().x, new_points0.front().y);
            measurement.z1 = Eigen::Vector2d(new_points1.front().x, new_points1.front().y);

            //if (measurement.check_epipolar_constraint(stereo_camera_, epipolar_outlier_threshold_))
                measurement.set_initialized();
        }

        new_points0_dist.erase(new_points0_dist.begin());
        new_points1_dist.erase(new_points1_dist.begin());

        new_points0.erase(new_points0.begin());
        new_points1.erase(new_points1.begin());

        status.erase(status.begin());
    }
}

void FeatureTracker::visualize()
{
    std::vector<cv::Mat> color_images(2);

    cv::cvtColor(prev_image0_, color_images[0], CV_GRAY2BGR);
    cv::cvtColor(prev_image1_, color_images[1], CV_GRAY2BGR);

    for (size_t i = 0; i < curr_measurements_.size(); ++i)
    {
        if (curr_measurements_[i].is_invalid())
        {
            continue;
        }

        const cv::Point2f curr_point0(curr_measurements_[i].z0_dist.x(), curr_measurements_[i].z0_dist.y());
        const cv::Point2f curr_point1(curr_measurements_[i].z1_dist.x(), curr_measurements_[i].z1_dist.y());

        // select color based on state
        cv::Scalar color;
        if (curr_measurements_[i].is_tracked())
        {
            color = cv::Scalar(0, 255, 0);

            const cv::Point2f prev_point0(prev_measurements_[i].z0_dist.x(), prev_measurements_[i].z0_dist.y());
            const cv::Point2f prev_point1(prev_measurements_[i].z1_dist.x(), prev_measurements_[i].z1_dist.y());

            cv::line(color_images[0], curr_point0, prev_point0, color);
            cv::line(color_images[1], curr_point1, prev_point1, color);
        }
        else
        {
            color = cv::Scalar(0, 0, 255);
        }

        // draw circle
        cv::circle(color_images[0], curr_point0, 1, color, 2);
        cv::circle(color_images[1], curr_point1, 1, color, 2);

    }
    cv::Scalar color;
    cv::Mat color_image;
    cv::hconcat(color_images, color_image);

    cv::imshow("FeatureTracker Visualization", color_image);
    cv::waitKey(1);
}