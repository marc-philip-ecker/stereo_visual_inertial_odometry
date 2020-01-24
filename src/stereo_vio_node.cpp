/**
 * @file stereo_vio.cpp
 * @author Marc-Philip Ecker
 * @date 23.10.19
 */

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Imu.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <cv_bridge/cv_bridge.h>
#include <stereo_vio/stereo_vio.h>
#include <geometry_msgs/PointStamped.h>

#define SKIP (0)

int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "vins_mono_node");
    ros::NodeHandle n;

    // Open ROS Bag
    ROS_INFO("Open ROS Bag...");
    rosbag::Bag bag;
    bag.open("/home/marc/Downloads/MH_04_difficult.bag", rosbag::BagMode::Read);

    // Read gt
    rosbag::View gt_view(bag, rosbag::TopicQuery("/leica/position"));
    nav_msgs::Path path;
    path.header.frame_id = "map";
    ros::Publisher gt_pub = n.advertise<nav_msgs::Path>("/stereo_vio/gt", 10);
    bool first = true;
    Eigen::Vector3d init;

    for (rosbag::MessageInstance const m_gt : gt_view)
    {
        geometry_msgs::PointStampedConstPtr gt_msg = m_gt.instantiate<geometry_msgs::PointStamped>();
        Eigen::Vector3d gt_pt(gt_msg->point.x, gt_msg->point.y, gt_msg->point.z);

        if (first)
        {
            init = gt_pt;
            first = false;
        }
        gt_pt = gt_pt - init;

        if (gt_msg != nullptr)
        {
            path.poses.emplace_back();
            path.poses.back().pose.position.x = gt_pt.x();
            path.poses.back().pose.position.y = gt_pt.y();
            path.poses.back().pose.position.z = gt_pt.z();
        }
    }

    // Read left images
    ROS_INFO("Read Images...");
    rosbag::View cam_left_view(bag, rosbag::TopicQuery("/cam0/image_raw"));
    std::vector<ros::Time> stamp_left;
    std::vector<cv::Mat> images_left;

    int skip = 0;
    for (rosbag::MessageInstance const m_cam : cam_left_view)
    {
        skip++;
        if (skip < SKIP) continue;

        sensor_msgs::ImageConstPtr image_msg = m_cam.instantiate<sensor_msgs::Image>();

        if (image_msg != nullptr)
        {
            stamp_left.emplace_back(image_msg->header.stamp);
            images_left.emplace_back(cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::MONO8)->image);
        }
    }

    // Read right images
    ROS_INFO("Read Images...");
    rosbag::View cam_right_view(bag, rosbag::TopicQuery("/cam1/image_raw"));
    std::vector<ros::Time> stamp_right;
    std::vector<cv::Mat> images_right;

    skip = 0;
    for (rosbag::MessageInstance const m_cam : cam_right_view)
    {
        skip++;
        if (skip < SKIP) continue;

        sensor_msgs::ImageConstPtr image_msg = m_cam.instantiate<sensor_msgs::Image>();

        if (image_msg != nullptr)
        {
            stamp_right.emplace_back(image_msg->header.stamp);
            images_right.emplace_back(cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::MONO8)->image);
        }
    }

    // Read IMU
    ROS_INFO("Read IMU Measurements...");
    rosbag::View imu_view(bag, rosbag::TopicQuery("/imu0"));
    std::vector<ros::Time> imu_stamps;
    std::vector<ImuMeasurement> u_vector(0);

    for (rosbag::MessageInstance const m_imu : imu_view)
    {
        sensor_msgs::ImuConstPtr imu_msg = m_imu.instantiate<sensor_msgs::Imu>();

        if (imu_msg->header.stamp < stamp_left.front())
        {
            continue;
        }

        if (imu_msg != nullptr)
        {
            imu_stamps.push_back(imu_msg->header.stamp);
            u_vector.emplace_back();

            u_vector.back().block<3, 1>(ACC_START, 0) = Eigen::Vector3d(imu_msg->linear_acceleration.x,
                                                                        imu_msg->linear_acceleration.y,
                                                                        imu_msg->linear_acceleration.z);
            u_vector.back().block<3, 1>(ANG_START, 0) = Eigen::Vector3d(imu_msg->angular_velocity.x,
                                                                        imu_msg->angular_velocity.y,
                                                                        imu_msg->angular_velocity.z);
        }
    }

    StereoVio vio(n);

    ROS_INFO("Start...");
    size_t imu_id = 0;
    size_t img_left_id = 0;
    size_t img_right_id = 0;

    stamp_left.erase(stamp_left.begin());
    stamp_right.erase(stamp_right.begin());
    images_left.erase(images_left.begin());
    images_right.erase(images_right.begin());

    size_t calls = 0;
    while (img_left_id != images_left.size() && img_right_id != images_right.size())
    {
        while (stamp_left[img_left_id] != stamp_right[img_right_id])
        {
            while (stamp_left[img_left_id] < stamp_right[img_right_id])
                img_left_id++;

            while (stamp_right[img_right_id] < stamp_left[img_left_id])
                img_right_id++;
        }

        std::vector<ros::Time> input_imu_stamps(0);
        std::vector<ImuMeasurement> u_vector_in(0);

        for (; imu_id < imu_stamps.size(); ++imu_id)
        {
            input_imu_stamps.push_back(imu_stamps[imu_id]);
            u_vector_in.emplace_back(u_vector[imu_id]);

            if (imu_stamps[imu_id] == stamp_left[img_left_id])
            {
                break;
            }
            else if (imu_stamps[imu_id] > stamp_left[img_left_id])
            {
                input_imu_stamps.back() = stamp_left[img_left_id];
                imu_id--;
                imu_stamps[imu_id] = stamp_left[img_left_id];
                break;
            }
        }

        vio.process_measurements(stamp_left[img_left_id], images_left[img_left_id], images_right[img_right_id],
                                 input_imu_stamps, u_vector_in);
        vio.visualize();
        gt_pub.publish(path);

        img_left_id++;
        img_right_id++;
    }

    ROS_INFO("Finished!");
}
