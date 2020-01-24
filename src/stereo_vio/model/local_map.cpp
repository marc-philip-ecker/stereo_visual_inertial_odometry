/**
 * @file local_map.cpp
 * @author Marc-Philip Ecker
 * @date 09.01.20
 */
#include "stereo_vio/model/local_map.h"

LocalMap::LocalMap() :
        landmarks(0),
        n_occurrences(0),
        free_ids_(0)
{

}

int LocalMap::add_landmark(const Eigen::Vector3d &landmark)
{
    int landmark_id;
    if (free_ids_.empty())
    {
        landmark_id = (int) landmarks.size();

        landmarks.push_back(landmark);
        n_occurrences.push_back(1);
    }
    else
    {
        landmark_id = free_ids_.front();
        free_ids_.erase(free_ids_.begin());

        landmarks[landmark_id] = landmark;
        n_occurrences[landmark_id] = 1;
    }

    return landmark_id;
}

void LocalMap::notify_observations(const std::vector<CameraMeasurement> &measurements)
{
    for (const auto &measurement : measurements)
    {
        if (measurement.landmark_id < 0)
            continue;

        n_occurrences[measurement.landmark_id]++;
    }
}

void LocalMap::notify_observations_removal(const std::vector<CameraMeasurement> &measurements)
{
    for (const auto &measurement : measurements)
    {
        if (measurement.landmark_id < 0)
            continue;

        n_occurrences[measurement.landmark_id]--;

        if (n_occurrences[measurement.landmark_id] == 0)
        {
            free_ids_.push_back(measurement.landmark_id);
        }
    }
}