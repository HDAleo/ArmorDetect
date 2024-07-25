#ifndef POSITION_H
#define POSITION_H

#include <cmath>
#include <iostream>
#include <vector>
#include "../include/number_classifier.hpp"

class Quaternion {
public:
    double w, x, y, z;
    Quaternion(double w, double x, double y, double z);
    Quaternion operator*(const Quaternion& q) const;
    Quaternion conjugate() const;
    static Quaternion fromEuler(double roll, double pitch, double yaw);
    void toEulerAngles(double& roll, double& pitch, double& yaw) const;
    void rotatePoint(double& px, double& py, double& pz) const;
};

class Pose {
public:
    double x, y, z;
    Quaternion orientation;

    Pose(double x, double y, double z, double roll, double pitch, double yaw);
    Pose(double x, double y, double z, const Quaternion& orientation);
    Pose transform(const Quaternion& rotation) const;
    void print() const;
};

Pose trans_final(const std::vector<Armor>& armors);
void draw_prediction(cv::Mat prediction, cv::Mat& img);

std::ostream& operator<<(std::ostream& os, const Quaternion& q);

#endif // POSITION_H
