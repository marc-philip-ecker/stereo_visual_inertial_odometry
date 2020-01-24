/**
 * @file parameters.h
 * @author Marc-Philip Ecker
 * @date 23.10.19
 */
#ifndef SRC_PARAMETERS_H
#define SRC_PARAMETERS_H

#define OPT_WINDOW_SIZE (10)
#define N_KEYFRAMES (OPT_WINDOW_SIZE - 5)
#define FEATURES_PER_IMAGE (100)
#define FEATURE_DISPLACEMENT (30)
#define EPIPOLAR_THRESHOLD (1e-4)
#define INIT_PARALLAX (40)

#define USE_KFS (0)

#define EUROC

#ifdef EUROC
#define R00_BODY_CAM0 (0.0148655429818)
#define R01_BODY_CAM0 (-0.999880929698)
#define R02_BODY_CAM0 (0.00414029679422)
#define R10_BODY_CAM0 (0.999557249008)
#define R11_BODY_CAM0 (0.0149672133247)
#define R12_BODY_CAM0 (0.025715529948)
#define R20_BODY_CAM0 (-0.0257744366974)
#define R21_BODY_CAM0 (0.00375618835797)
#define R22_BODY_CAM0 (0.999660727178)

#define T0_BODY_CAM0 (-0.0216401454975)
#define T1_BODY_CAM0 (-0.064676986768)
#define T2_BODY_CAM0 (0.00981073058949)

#define R00_BODY_CAM1 (0.0125552670891)
#define R01_BODY_CAM1 (-0.999755099723)
#define R02_BODY_CAM1 (0.0182237714554)
#define R10_BODY_CAM1 (0.999598781151)
#define R11_BODY_CAM1 (0.0130119051815)
#define R12_BODY_CAM1 (0.0251588363115)
#define R20_BODY_CAM1 (-0.0253898008918)
#define R21_BODY_CAM1 (0.0179005838253)
#define R22_BODY_CAM1 (0.999517347078)

#define T0_BODY_CAM1 (-0.0198435579556)
#define T1_BODY_CAM1 (0.0453689425024)
#define T2_BODY_CAM1 (0.00786212447038)

#define DIST0_CAM0 (-0.28340811)
#define DIST1_CAM0 (0.07395907)
#define DIST2_CAM0 (0.00019359)
#define DIST3_CAM0 (1.76187114e-05)

#define DIST0_CAM1 (-0.28368365)
#define DIST1_CAM1 (0.07451284)
#define DIST2_CAM1 (-0.00010473)
#define DIST3_CAM1 (-3.55590700e-05)

#define FX_CAM0 (458.654)
#define FY_CAM0 (457.296)
#define CX_CAM0 (367.215)
#define CY_CAM0 (248.375)

#define FX_CAM1 (457.587)
#define FY_CAM1 (456.134)
#define CX_CAM1 (379.999)
#define CY_CAM1 (255.238)

#define GYRO_NOISE_DENSITY (1.6968e-04)
#define GYRO_RANDOM_WALK (1.9393e-05)

#define ACCL_NOISE_DENSITY (2.0000e-3)
#define ACCL_RANDOM_WALK (3.0000e-3)

#define IMAGE_NOISE (1)

#endif

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

const cv::Mat CAMERA_MATRIX_CAM0 = (cv::Mat_<double>(3, 3) << FX_CAM0, 0, CX_CAM0, 0, FY_CAM0, CY_CAM0, 0, 0, 1);

const cv::Mat CAMERA_MATRIX_CAM1 = (cv::Mat_<double>(3, 3) << FX_CAM1, 0, CX_CAM1, 0, FY_CAM1, CY_CAM1, 0, 0, 1);

const cv::Mat DIST_COEFFS_CAM0 = (cv::Mat_<double>(4, 1) << DIST0_CAM0, DIST1_CAM0, DIST2_CAM0, DIST3_CAM0);

const cv::Mat DIST_COEFFS_CAM1 = (cv::Mat_<double>(4, 1) << DIST0_CAM1, DIST1_CAM1, DIST2_CAM1, DIST3_CAM1);

const Eigen::Matrix3d R_BODY_CAM0 = (const Eigen::Matrix3d)
        (Eigen::Matrix3d() << R00_BODY_CAM0, R01_BODY_CAM0, R02_BODY_CAM0,
                R10_BODY_CAM0, R11_BODY_CAM0, R12_BODY_CAM0,
                R20_BODY_CAM0, R21_BODY_CAM0, R22_BODY_CAM0).finished();

const Eigen::Matrix3d R_BODY_CAM1 = (const Eigen::Matrix3d)
        (Eigen::Matrix3d() << R00_BODY_CAM1, R01_BODY_CAM1, R02_BODY_CAM1,
                R10_BODY_CAM1, R11_BODY_CAM1, R12_BODY_CAM1,
                R20_BODY_CAM1, R21_BODY_CAM1, R22_BODY_CAM1).finished();

const Eigen::Vector3d T_BODY_CAM0 = (const Eigen::Vector3d)
        (Eigen::Vector3d() << T0_BODY_CAM0, T1_BODY_CAM0, T2_BODY_CAM0).finished();

const Eigen::Vector3d T_BODY_CAM1 = (const Eigen::Vector3d)
        (Eigen::Vector3d() << T0_BODY_CAM1, T1_BODY_CAM1, T2_BODY_CAM1).finished();

const Eigen::Quaterniond Q_BODY_CAM0(R_BODY_CAM0);

const Eigen::Quaterniond Q_BODY_CAM1(R_BODY_CAM1);

#endif //SRC_PARAMETERS_H
