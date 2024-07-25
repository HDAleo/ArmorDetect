#include "../include/kalman.hpp"
#include "../include/position.hpp"

KalmanFilter3D::KalmanFilter3D() {
    KF.init(6, 6, 0); // 状态向量为 (x, y, z, roll, pitch, yaw)，观测向量为 (x, y, z, roll, pitch, yaw)

    // 初始化卡尔曼滤波器
    KF.transitionMatrix = (cv::Mat_<float>(6, 6) <<
                           1, 0, 0, 1, 0, 0,
                           0, 1, 0, 0, 1, 0,
                           0, 0, 1, 0, 0, 1,
                           0, 0, 0, 1, 0, 0,
                           0, 0, 0, 0, 1, 0,
                           0, 0, 0, 0, 0, 1);

    setIdentity(KF.measurementMatrix); // 观测矩阵 H
    setIdentity(KF.processNoiseCov, cv::Scalar::all(1e-1)); // 过程噪声协方差 Q
    setIdentity(KF.measurementNoiseCov, cv::Scalar::all(4e-1)); // 观测噪声协方差 R
}

KalmanFilter3D::~KalmanFilter3D() {}

void KalmanFilter3D::init(const Pose& pose) {
    KF.statePre.at<float>(0) = pose.x; // 初始 x 坐标
    KF.statePre.at<float>(1) = pose.y; // 初始 y 坐标
    KF.statePre.at<float>(2) = pose.z; // 初始 z 坐标
    double roll, pitch, yaw;
    pose.orientation.toEulerAngles(roll, pitch, yaw);
    KF.statePost.at<float>(3) = roll;             // 更新 roll 角
    KF.statePost.at<float>(4) = pitch;            // 更新 pitch 角
    KF.statePost.at<float>(5) = yaw;              // 更新 yaw 角
}

cv::Mat KalmanFilter3D::predict() {
    return KF.predict();
}

// 添加一个方法用于更新状态向量和执行预测
cv::Mat KalmanFilter3D::updateAndPredict(const Pose& pose,cv::Mat& frame) {
    // 更新卡尔曼滤波器的状态向量 statePost
    KF.statePost.at<float>(0) = pose.x;      // 更新 x 坐标
    KF.statePost.at<float>(1) = pose.y;      // 更新 y 坐标
    KF.statePost.at<float>(2) = pose.z;      // 更新 z 坐标

    double roll, pitch, yaw;
    pose.orientation.toEulerAngles(roll, pitch, yaw);
    KF.statePost.at<float>(3) = roll;             // 更新 roll 角
    KF.statePost.at<float>(4) = pitch;            // 更新 pitch 角
    KF.statePost.at<float>(5) = yaw;              // 更新 yaw 角

    // 执行卡尔曼滤波器的预测步骤
    cv::Mat prediction =KF.predict();

    draw_prediction(prediction,frame);
    return prediction;
}

