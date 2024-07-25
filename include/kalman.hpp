#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP

#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/video/tracking.hpp>
#include "../include/position.hpp"

class KalmanFilter3D {
public:
    KalmanFilter3D();
    ~KalmanFilter3D();

    void init(const Pose& pose);
    cv::Mat predict();

    cv::Mat updateAndPredict(const Pose& pose,cv::Mat& frame);

private:
    cv::KalmanFilter KF;
};

#endif // KALMAN_FILTER_HPP
