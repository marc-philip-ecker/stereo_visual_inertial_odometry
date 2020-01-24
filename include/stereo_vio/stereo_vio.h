/**
 * @file stereo_vio.h
 * @author Marc-Philip Ecker
 * @date 23.10.19
 */
#ifndef SRC_STEREO_VIO_H
#define SRC_STEREO_VIO_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud.h>
#include <stereo_vio/processing/feature_tracker.h>
#include <stereo_vio/processing/pre_integrator.h>
#include <stereo_vio/model/camera_measurement.h>
#include <stereo_vio/model/local_map.h>

class StereoVio
{
public:
    explicit StereoVio(const ros::NodeHandle &n);

    void process_measurements(const ros::Time &image_stamp,
                              const cv::Mat &image0,
                              const cv::Mat &image1,
                              const std::vector<ros::Time> &imu_stamps,
                              const std::vector<ImuMeasurement> &u_vector);

    void visualize();

private:
    ros::NodeHandle n_;

    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
     * Processing Algorithms                                                                                         *
     * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

    FeatureTracker tracker_;

    PreIntegrator integrator_;

    StereoCamera stereo_camera_;

    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
     * Processing Data                                                                                               *
     * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

    std::vector<State> x_window_;

    std::vector<ControlInput> u_window_;

    std::vector<Eigen::Matrix<double, CONTROL_INPUT_SIZE + 5, CONTROL_INPUT_SIZE>> Q_window_;

    std::vector<std::vector<CameraMeasurement>> measurements_window_;

    Eigen::Matrix2d R_i;

    LocalMap map_;

    bool initialized_;

    std::map<int, double> chi_square_table_;

    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
     * Publisher                                                                                                     *
     * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

    ros::Publisher path_publisher_;

    ros::Publisher pose_publisher_;

    ros::Publisher landmarks_publisher_;

    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
     * ROS Message Objects                                                                                           *
     * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

    nav_msgs::Path path_msg_;

    sensor_msgs::PointCloud landmarks_msg_;

    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
     * Private Functions                                                                                             *
     * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

    void process_image(const cv::Mat &image0, const cv::Mat &image1);

    void initialize();

    void triangulate_points();

    void optimize();

    void visualize_stereo_images();

    void visualize_landmarks();

    void visualize_path();
};

#endif //SRC_STEREO_VIO_H
