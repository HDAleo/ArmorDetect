#ifndef _DETECTOR_H_
#define _DETECTOR_H_

#include<opencv2/opencv.hpp>
#include "../include/number_classifier.hpp"

using namespace std;
using namespace cv;

class Detect{
    public:
    Mat preprocess(Mat img);
    vector<RotatedRect> Findcounter(Mat pre);
    vector<RotatedRect> detect(vector<RotatedRect> processedImage);
    vector<Point2f> point_2d(RotatedRect pose);
    vector<Point3f> point_3d();

    Mat cameraMatrix = (Mat_<double>(3, 3) << 2102.080562187802, 0.000000000000, 689.2057889332623, 0.000000000000, 2094.0179120166754, 529.83878, 0.000000000000, 0.000000000000, 1.000000000000);
    Mat distCoeffs = (Mat_<double>(1, 5) << -0.06478109387525666,  0.39036067923005396, -0.0042514793151166306, 0.008306749648029776,  -1.6613800909405605);
    Mat rvec, tvec;
    vector<Point2f> Points2d;
    vector<Point3f> Points3d;

    vector<Armor> detectArmor(Mat& img);
};

#endif