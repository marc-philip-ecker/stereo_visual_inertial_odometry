/**
 * @file local_map.h
 * @author Marc-Philip Ecker
 * @date 09.01.20
 */
#ifndef SRC_LOCAL_MAP_H
#define SRC_LOCAL_MAP_H

#include <Eigen/Dense>
#include <vector>

#include <stereo_vio/model/camera_measurement.h>

class LocalMap
{
public:
    std::vector<Eigen::Vector3d> landmarks;

    std::vector<size_t> n_occurrences;

    LocalMap();

    int add_landmark(const Eigen::Vector3d &landmark);

    void notify_observations(const std::vector<CameraMeasurement> &measurements);

    void notify_observations_removal(const std::vector<CameraMeasurement> &measurements);

private:
    std::vector<size_t> free_ids_;
};
#endif //SRC_LOCAL_MAP_H
