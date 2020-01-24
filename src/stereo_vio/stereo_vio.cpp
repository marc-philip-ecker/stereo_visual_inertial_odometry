/**
 * @file stereo_vio.cpp
 * @author Marc-Philip Ecker
 * @date 23.10.19
 */
#include "stereo_vio/stereo_vio.h"
#include <stereo_vio/parameters.h>
#include <stereo_vio/ceres/orientation_local_parameterization.h>
#include <stereo_vio/ceres/reprojection_cost_function.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <cxeigen.hpp>
#include <boost/math/distributions/chi_squared.hpp>
#include <ceres/ceres.h>
#include <stereo_vio/ceres/imu_cost_function.h>
#include <stereo_vio/processing/util.h>

StereoVio::StereoVio(const ros::NodeHandle &n) :
        n_(n),
        stereo_camera_(Camera(T_BODY_CAM0, Q_BODY_CAM0, CAMERA_MATRIX_CAM0, DIST_COEFFS_CAM0),
                       Camera(T_BODY_CAM1, Q_BODY_CAM1, CAMERA_MATRIX_CAM1, DIST_COEFFS_CAM1)),
        tracker_(StereoCamera(Camera(T_BODY_CAM0, Q_BODY_CAM0, CAMERA_MATRIX_CAM0, DIST_COEFFS_CAM0),
                              Camera(T_BODY_CAM1, Q_BODY_CAM1, CAMERA_MATRIX_CAM1, DIST_COEFFS_CAM1)),
                 FEATURES_PER_IMAGE,
                 FEATURE_DISPLACEMENT,
                 EPIPOLAR_THRESHOLD),
        integrator_(ACCL_NOISE_DENSITY, ACCL_RANDOM_WALK, GYRO_NOISE_DENSITY, GYRO_RANDOM_WALK),
        measurements_window_(0),
        initialized_(false)
{
    R_i = Eigen::Matrix2d::Identity();

    path_publisher_ = n_.advertise<nav_msgs::Path>("stereo_vio/path", 10);
    pose_publisher_ = n_.advertise<geometry_msgs::PoseWithCovarianceStamped>("stereo_vio/pose", 10);
    landmarks_publisher_ = n_.advertise<sensor_msgs::PointCloud>("stereo_vio/landmarks", 10);

    for (int i = 1; i < 100; ++i)
    {
        boost::math::chi_squared chi_sq(i);
        chi_square_table_[i] = boost::math::quantile(chi_sq, 0.05);
    }
}

void StereoVio::process_measurements(const ros::Time &image_stamp,
                                     const cv::Mat &image0,
                                     const cv::Mat &image1,
                                     const std::vector<ros::Time> &imu_stamps,
                                     const std::vector<ImuMeasurement> &u_vector)
{
    const auto &start = ros::Time::now();

    process_image(image0, image1);

    u_window_.emplace_back();
    u_window_.back().stamps = imu_stamps;
    u_window_.back().measurements = u_vector;
    integrator_.u() = u_window_.back();
    integrator_.pre_integrate();
    u_window_.back() = integrator_.u();

    if (x_window_.empty())
    {
        x_window_.emplace_back();

        triangulate_points();

#if 0
        Eigen::Vector3d sum(0.0, 0.0, 0.0);
        for (auto &u : u_vector)
        {
            sum += u.block<3, 1>(0, 0);
        }
        sum /= u_vector.size();

        x_window_.back().gravity() = -9.81 * sum.normalized();

        Eigen::Quaterniond q_w_b0 = Eigen::Quaterniond::FromTwoVectors(x_window_.back().gravity().normalized(),
                                                                       -Eigen::Vector3d::UnitZ());

        for (auto &x : x_window_)
        {
            x.t_world_body() = q_w_b0 * x.t_world_body();
            x.q_world_body() = q_w_b0 * x.q_world_body();
            x.v_world_body() = q_w_b0 * x.v_world_body();
            x.gravity() = 9.81 * -Eigen::Vector3d::UnitZ();
        }

        for (auto &lm : map_.landmarks)
        {
            lm = q_w_b0 * lm;
        }
#endif

    }
    else
    {
        // State propagation
        x_window_.push_back(x_window_.back());

        if (initialized_)
        {
            double dt = (u_window_.back().stamps.back() - u_window_.back().stamps.front()).toSec();

            x_window_.back().t_world_body() += x_window_.back().v_world_body() * dt
                                               - 0.5 * 9.81 * Eigen::Vector3d::UnitZ() * dt * dt
                                               + x_window_.back().q_world_body() * u_window_.back().alpha();
            x_window_.back().v_world_body() += -9.81 * Eigen::Vector3d::UnitZ() * dt
                                               + x_window_.back().q_world_body() * u_window_.back().beta();
            x_window_.back().q_world_body() = x_window_.back().q_world_body() * u_window_.back().gamma();
        }

        // Remove frames (oldest if the oldest in the sliding window is no keyframe and oldest in sw otherwise).
        while (initialized_ && x_window_.size() > OPT_WINDOW_SIZE)
        {
#if USE_KFS
            size_t tracked_cnt = 0;
            for (auto &measurement : measurements_window_[N_KEYFRAMES])
            {
                if (measurement.is_tracked())
                {
                    tracked_cnt++;
                }
            }

            if (tracked_cnt < FEATURES_PER_IMAGE / 2)
            {
                // is keyframe
                map_.notify_observations_removal(measurements_window_.front());
                x_window_.erase(x_window_.begin());
                measurements_window_.erase(measurements_window_.begin());
                u_window_.erase(u_window_.begin());
            }
            else
            {
                // is not keyframe
                std::vector<CameraMeasurement> &prev_measurements = measurements_window_[N_KEYFRAMES];
                std::vector<CameraMeasurement> &curr_measurements = measurements_window_[N_KEYFRAMES + 1];
                for (size_t m_id = 0; m_id < FEATURES_PER_IMAGE; ++m_id)
                {
                    if (!prev_measurements[m_id].is_tracked() && curr_measurements[m_id].is_tracked())
                    {
                        curr_measurements[m_id].set_initialized();
                    }
                }

                u_window_[N_KEYFRAMES + 1].stamps.insert(u_window_[N_KEYFRAMES + 1].stamps.end(),
                                                         u_window_[N_KEYFRAMES].stamps.begin(),
                                                         u_window_[N_KEYFRAMES].stamps.end());
                u_window_[N_KEYFRAMES + 1].measurements.insert(u_window_[N_KEYFRAMES + 1].measurements.end(),
                                                               u_window_[N_KEYFRAMES].measurements.begin(),
                                                               u_window_[N_KEYFRAMES].measurements.end());

                integrator_.u() = u_window_[N_KEYFRAMES + 1];
                integrator_.x() = x_window_[N_KEYFRAMES + 1];
                integrator_.pre_integrate();
                u_window_[N_KEYFRAMES + 1] = integrator_.u();

                map_.notify_observations_removal(measurements_window_[N_KEYFRAMES]);
                x_window_.erase(x_window_.begin() + N_KEYFRAMES);
                measurements_window_.erase(measurements_window_.begin() + N_KEYFRAMES);
                u_window_.erase(u_window_.begin() + N_KEYFRAMES);
            }
#else
            map_.notify_observations_removal(measurements_window_.front());
            x_window_.erase(x_window_.begin());
            measurements_window_.erase(measurements_window_.begin());
            u_window_.erase(u_window_.begin());
#endif
        }

        optimize();
        triangulate_points();

        if (!initialized_ && x_window_.size() == 10)
        {
            initialize();
        }
    }

    const auto &end = ros::Time::now();

    ROS_INFO("Processing Time: %fms", (end - start).toSec() * 1e3);
}

void StereoVio::initialize()
{
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(6 * (x_window_.size() - 1), 3 * x_window_.size() + 3);
    Eigen::VectorXd y = Eigen::VectorXd::Zero(H.rows());

    for (size_t state_id = 0; state_id < x_window_.size() - 1; ++state_id)
    {
        const State &x_k = x_window_[state_id];
        const State &x_kp1 = x_window_[state_id + 1];

        const Eigen::Vector3d &alpha = u_window_[state_id + 1].alpha();
        const Eigen::Vector3d &beta = u_window_[state_id + 1].beta();
        const double dt = (u_window_[state_id + 1].stamps.back() - u_window_[state_id + 1].stamps.front()).toSec();

        y.block<3, 1>(state_id * 6, 0) = x_kp1.t_world_body() - x_k.t_world_body() - x_k.q_world_body() * alpha;
        y.block<3, 1>(state_id * 6 + 3, 0) = x_k.q_world_body() * beta;

        H.block<3, 3>(state_id * 6, state_id * 3) = Eigen::Matrix3d::Identity() * dt;
        H.block<3, 3>(state_id * 6, H.cols() - 3) = Eigen::Matrix3d::Identity() * dt * dt;

        H.block<3, 3>(state_id * 6 + 3, state_id * 3) = -Eigen::Matrix3d::Identity();
        H.block<3, 3>(state_id * 6 + 3, (state_id + 1) * 3) = Eigen::Matrix3d::Identity();
        H.block<3, 3>(state_id * 6 + 3, H.cols() - 3) = -Eigen::Matrix3d::Identity() * dt;
    }

    const Eigen::MatrixXd Ht = H.transpose();
    const Eigen::MatrixXd HtH = H.transpose() * H;
    const Eigen::VectorXd Hty = H.transpose() * y;

    const Eigen::VectorXd r = HtH.ldlt().solve(Hty);

    const Eigen::Vector3d g = r.block<3, 1>(H.cols() - 3, 0);
    const Eigen::Quaterniond q_w_b0 = Eigen::Quaterniond::FromTwoVectors(g.normalized(), -Eigen::Vector3d::UnitZ());

    for (size_t state_id = 0; state_id < x_window_.size(); ++state_id)
    {
        State &x_k = x_window_[state_id];

        x_k.t_world_body() = q_w_b0 * x_k.t_world_body();
        x_k.v_world_body() = q_w_b0 * r.block<3, 1>(state_id * 3, 0);
        x_k.q_world_body() = q_w_b0 * x_k.q_world_body();
    }

    for (Eigen::Vector3d &l_w : map_.landmarks)
    {
        l_w = q_w_b0 * l_w;
    }

    initialized_ = true;
}

void StereoVio::optimize()
{
    ceres::Problem problem;
    ceres::LocalParameterization *orientation_parameterization = new OrientationLocalParameterization();

    for (size_t state_id = 0; state_id < x_window_.size(); ++state_id)
    {
        problem.AddParameterBlock(x_window_[state_id].t_world_body_ptr(), 3);
        problem.AddParameterBlock(x_window_[state_id].v_world_body_ptr(), 3);
        problem.AddParameterBlock(x_window_[state_id].q_world_body_ptr(), 4, orientation_parameterization);
        problem.AddParameterBlock(x_window_[state_id].bias_a_ptr(), 3);
        problem.AddParameterBlock(x_window_[state_id].bias_w_ptr(), 3);

        if ((x_window_.size() <= N_KEYFRAMES && state_id < 1) ||
            (x_window_.size() > N_KEYFRAMES && state_id < N_KEYFRAMES))
        {
            problem.SetParameterBlockConstant(x_window_[state_id].t_world_body_ptr());
            problem.SetParameterBlockConstant(x_window_[state_id].q_world_body_ptr());
        }

        if (initialized_ && ((x_window_.size() <= N_KEYFRAMES && state_id >= 1) ||
                             (x_window_.size() > N_KEYFRAMES && state_id >= N_KEYFRAMES)))
        {
            ceres::CostFunction *cost_function_imu = new ImuCostFunction(u_window_[state_id]);
            problem.AddResidualBlock(cost_function_imu, NULL,
                                     x_window_[state_id - 1].t_world_body_ptr(),
                                     x_window_[state_id - 1].v_world_body_ptr(),
                                     x_window_[state_id - 1].q_world_body_ptr(),
                                     x_window_[state_id - 1].bias_a_ptr(),
                                     x_window_[state_id - 1].bias_w_ptr(),
                                     x_window_[state_id].t_world_body_ptr(),
                                     x_window_[state_id].v_world_body_ptr(),
                                     x_window_[state_id].q_world_body_ptr(),
                                     x_window_[state_id].bias_a_ptr(),
                                     x_window_[state_id].bias_w_ptr());
        }

        for (size_t obs_id = 0; obs_id < measurements_window_[state_id].size(); ++obs_id)
        {
            int landmark_id = measurements_window_[state_id][obs_id].landmark_id;

            if (landmark_id < 0)
                continue;

            if (map_.n_occurrences[landmark_id] < 2)
                continue;

            problem.AddParameterBlock(map_.landmarks[landmark_id].data(), 3);

            ceres::CostFunction *cost_function_left = new ReprojectionCostFunction(
                    measurements_window_[state_id][obs_id].z0,
                    R_i,
                    Q_BODY_CAM0,
                    T_BODY_CAM0,
                    FX_CAM0, FY_CAM0, CX_CAM0, CY_CAM0);
            ceres::CostFunction *cost_function_right = new ReprojectionCostFunction(
                    measurements_window_[state_id][obs_id].z1,
                    R_i,
                    Q_BODY_CAM1,
                    T_BODY_CAM1,
                    FX_CAM1, FY_CAM1, CX_CAM1, CY_CAM1);

            problem.AddResidualBlock(cost_function_left, new ceres::CauchyLoss(1.0),
                                     x_window_[state_id].q_world_body_ptr(),
                                     x_window_[state_id].t_world_body_ptr(),
                                     map_.landmarks[landmark_id].data());
            problem.AddResidualBlock(cost_function_right, new ceres::CauchyLoss(1.0),
                                     x_window_[state_id].q_world_body_ptr(),
                                     x_window_[state_id].t_world_body_ptr(),
                                     map_.landmarks[landmark_id].data());
        }
    }

    // create ceres options
    ceres::Solver::Options options;
    options.num_threads = 4;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    //options.max_num_iterations = 5;
    // options.max_solver_time_in_seconds = 0.2;

    // create summars
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    for (int i = 0; i < x_window_.size(); ++i)
    {
        integrator_.x() = x_window_[i];
        integrator_.u() = u_window_[i];
        integrator_.pre_integrate();

        u_window_[i] = integrator_.u();
    }
}

void StereoVio::process_image(const cv::Mat &image0, const cv::Mat &image1)
{
    // Process Images
    tracker_.process_image_pair(image0, image1);
    // Get measurements
    std::vector<CameraMeasurement> new_measurements = tracker_.curr_measurements();

    for (size_t i = 0; i < new_measurements.size(); ++i)
    {
        if (new_measurements[i].is_tracked())
        {
            new_measurements[i].landmark_id = measurements_window_.back()[i].landmark_id;
        }
    }

    map_.notify_observations(new_measurements);
    measurements_window_.push_back(new_measurements);
}

void StereoVio::triangulate_points()
{
    std::vector<CameraMeasurement> invalid_measurements(0);
    for (auto &measurement : measurements_window_.back())
    {


        if (measurement.is_invalid() || measurement.landmark_id >= 0)
        {
            continue;
        }

        if ((measurement.z0 - measurement.z1).norm() < 1)
            continue;

        const Eigen::Vector3d l_b = measurement.stereo_triangulate(stereo_camera_);
        const Eigen::Vector3d l_c0 =
                stereo_camera_.cam0.q_body_cam.conjugate() * (l_b - stereo_camera_.cam0.t_body_cam);

        if (l_c0.z() < 0)
            continue;

        measurement.landmark_id = map_.add_landmark(
                x_window_.back().q_world_body() * l_b + x_window_.back().t_world_body());
    }
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Visualization                                                                                                     *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

void StereoVio::visualize()
{
#if 0
    visualize_landmarks();
    visualize_stereo_images();
#endif

#if 1
    if (initialized_)
    {
        visualize_path();
        visualize_landmarks();
    }
#else
    visualize_path();
    visualize_landmarks();
#endif
}

void StereoVio::visualize_landmarks()
{
    landmarks_msg_.points.resize(0);
    landmarks_msg_.header.stamp = ros::Time::now();
    landmarks_msg_.header.frame_id = "map";

    for (const auto &measurement : measurements_window_.back())
    {
        if (measurement.landmark_id < 0)
        {
            continue;
        }

        const auto &l = map_.landmarks[measurement.landmark_id];
        landmarks_msg_.points.emplace_back();

        landmarks_msg_.points.back().x = l.x();
        landmarks_msg_.points.back().y = l.y();
        landmarks_msg_.points.back().z = l.z();
    }

    landmarks_publisher_.publish(landmarks_msg_);
}

void StereoVio::visualize_path()
{
#if 0
    path_msg_.header.stamp = ros::Time::now();
    path_msg_.header.frame_id = "map";

    path_msg_.poses.emplace_back();
    path_msg_.poses.back().header.stamp = path_msg_.header.stamp;
    path_msg_.poses.back().header.frame_id = "map";

    const Eigen::Quaterniond &q_world_body = body_states_.back().q_world_body;
    const Eigen::Vector3d &t_world_body = body_states_.back().t_world_body;

    path_msg_.poses.back().pose.position.x = t_world_body.x();
    path_msg_.poses.back().pose.position.y = t_world_body.y();
    path_msg_.poses.back().pose.position.z = t_world_body.z();

    path_msg_.poses.back().pose.orientation.w = q_world_body.w();
    path_msg_.poses.back().pose.orientation.x = q_world_body.x();
    path_msg_.poses.back().pose.orientation.y = q_world_body.y();
    path_msg_.poses.back().pose.orientation.z = q_world_body.z();

    path_publisher_.publish(path_msg_);
#endif
    geometry_msgs::PoseWithCovarianceStamped ros_pose;

    ros_pose.header.frame_id = "map";
    ros_pose.header.stamp = ros::Time::now();

    ros_pose.pose.pose.position.x = x_window_.back().t_world_body().x();
    ros_pose.pose.pose.position.y = x_window_.back().t_world_body().y();
    ros_pose.pose.pose.position.z = x_window_.back().t_world_body().z();

    ros_pose.pose.pose.orientation.w = x_window_.back().q_world_body().w();
    ros_pose.pose.pose.orientation.x = x_window_.back().q_world_body().x();
    ros_pose.pose.pose.orientation.y = x_window_.back().q_world_body().y();
    ros_pose.pose.pose.orientation.z = x_window_.back().q_world_body().z();

    path_msg_.header.frame_id = "map";
    path_msg_.header.stamp = ros_pose.header.stamp;

    path_msg_.poses.emplace_back();
    path_msg_.poses.back().header.frame_id = "map";
    path_msg_.poses.back().header.stamp = ros_pose.header.stamp;

    path_msg_.poses.back().pose.position.x = x_window_.back().t_world_body().x();
    path_msg_.poses.back().pose.position.y = x_window_.back().t_world_body().y();
    path_msg_.poses.back().pose.position.z = x_window_.back().t_world_body().z();

    path_msg_.poses.back().pose.orientation.w = x_window_.back().q_world_body().w();
    path_msg_.poses.back().pose.orientation.x = x_window_.back().q_world_body().x();
    path_msg_.poses.back().pose.orientation.y = x_window_.back().q_world_body().y();
    path_msg_.poses.back().pose.orientation.z = x_window_.back().q_world_body().z();

    pose_publisher_.publish(ros_pose);
    path_publisher_.publish(path_msg_);
}
