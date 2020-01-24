/**
 * @file state.h
 * @author Marc-Philip Ecker
 * @date 21.12.19
 */
#ifndef SRC_MODEL_H
#define SRC_MODEL_H

#define ACC_START (0)
#define ANG_START (3)

#define MEASUREMENT_SIZE (6)


#include <Eigen/Dense>

typedef Eigen::Matrix<double, MEASUREMENT_SIZE, 1> ImuMeasurement;

#endif //SRC_MODEL_H
