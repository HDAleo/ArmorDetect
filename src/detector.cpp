#include "../include/detector.hpp"
#include "../include/number_classifier.hpp"

// 预处理图像
Mat Detect::preprocess(Mat img){
    Mat img_copy;
    undistort(img, img_copy, cameraMatrix, distCoeffs);
    vector<Mat> channels;
    split(img_copy, channels);
    Mat img_gray = channels.at(0);
    int value = 200;
    Mat dst;
    threshold(img_gray, dst, value, 255, THRESH_BINARY);
    Mat pre;
    Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(3, 3), Point(-1, -1));
    morphologyEx(dst, pre, MORPH_OPEN, kernel);
    return pre;
}

// 查找轮廓
vector<RotatedRect> Detect::Findcounter(Mat pre){
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(pre, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(-1, -1));
    vector<RotatedRect> processedImage;
    for(size_t i = 0; i < contours.size(); ++i){
        RotatedRect processedImage_rect = minAreaRect(contours[i]); // 求点集最小的矩形
        double width = processedImage_rect.size.width;              // 求矩形的宽度
        double height = processedImage_rect.size.height;            // 求矩形的高度
        double area = width * height;                      // 求矩形的面积

        //灯条判断，要改参数
        if (area > 200 && width / height >1 && width / height <7) {
            processedImage.push_back(processedImage_rect);
        }
    }
    return processedImage;
}

// 装甲板识别
vector<RotatedRect> Detect::detect(vector<RotatedRect> processedImage)
{
    vector<RotatedRect> armor_final;
    if (processedImage.size() < 2) return armor_final; // 如果检测到灯条数量小于两个，则直接返回
    
    for (size_t i = 0; i < processedImage.size() - 1; i++) {
        for (size_t j = i + 1; j < processedImage.size(); j++) {
            double angle_differ = abs(processedImage[i].angle - processedImage[j].angle);

            //灯条判断，要改参数
            bool if1 = (angle_differ < 10);
            bool if2 = (abs(processedImage[i].center.y - processedImage[j].center.y) < 20);
            bool if3 = (abs(processedImage[i].center.x-processedImage[j].center.x)<400);
            if (if1 && if2 && if3) {
                Point2f center = (processedImage[i].center + processedImage[j].center) / 2;
                // 两灯条的中心连线方向向量
                Point2f diff = processedImage[j].center - processedImage[i].center;
                double armor_angle = atan2(diff.y, diff.x) * 180.0 / CV_PI;
                // 宽度和高度
                double armor_width = norm(diff);
                double armor_height = (processedImage[i].size.width + processedImage[j].size.width)/2;
                // 生成装甲板的旋转矩形
                RotatedRect armor(center, Size2f(armor_width, armor_height), armor_angle);
                armor_final.push_back(armor);
            }
        }
    }
    return armor_final;
}

// 二维坐标点加画矩形
vector<Point2f> Detect::point_2d(RotatedRect pose) {
    Point2f pt[4];
    vector<Point2f> point2d;
    pose.points(pt);

    // 按顺时针顺序排序：左下、左上、右上、右下
    point2d.push_back(pt[0]);
    point2d.push_back(pt[1]);
    point2d.push_back(pt[2]);
    point2d.push_back(pt[3]);

    // 将获取到的二维坐标存入vector  
    return point2d;           
}

// 获取三维坐标点
vector<Point3f> Detect::point_3d() {
    vector<Point3f> Points3d;
    Points3d.push_back(Point3f(0, 0, 0));
    Points3d.push_back(Point3f(0, 5.5, 0));
    Points3d.push_back(Point3f(13.5, 5.5, 0));
    Points3d.push_back(Point3f(13.5, 0, 0));
    return Points3d;
}

// 检测装甲板
vector<Armor> Detect::detectArmor(Mat& img) {
    Detect detect;
    Mat img_clone = img.clone();
    Mat pre = detect.preprocess(img_clone);
    vector<RotatedRect> processedImage = detect.Findcounter(pre);
    //imshow("Image", pre);
    vector<RotatedRect> armor_final = detect.detect(processedImage);
    vector<Armor> armors;
    vector<Armor> final_armors;

    // 初始化数字识别器
    std::string model_path = "../data/model/mlp.onnx";
    std::string label_path = "../data/model/label.txt";
    float confidence_threshold = 0.0;
    std::vector<std::string> ignore_classes = {};
    armor::NumberClassifier number_classifier(model_path, label_path, confidence_threshold, ignore_classes);

    // 计算三维点
    detect.Points3d = detect.point_3d();

    for (size_t i = 0; i < armor_final.size(); i++) {
        // 获取装甲板的二维坐标
        detect.Points2d = detect.point_2d(armor_final[i]);

        // 创建 Armor 对象并赋值
        Armor armor(detect.Points2d[0], detect.Points2d[1], detect.Points2d[3], detect.Points2d[2]);
        //PnP计算装甲板
        solvePnP(detect.Points3d, detect.Points2d, detect.cameraMatrix, detect.distCoeffs, detect.rvec, detect.tvec, false, SOLVEPNP_ITERATIVE);
        armor.rvec=detect.rvec;
        armor.tvec=detect.tvec;
        armors.push_back(armor);
    }

    number_classifier.ExtractNumbers(img_clone, armors);
    number_classifier.Classify(armors);

    for (const auto &armor : armors) {
        if(armor.classification_result!="negative"){
            final_armors.push_back(armor);
            circle(img, armor.left_light.bottom, 5, Scalar(0, 0, 255), -1);
            circle(img, armor.left_light.top, 5, Scalar(0, 0, 255), -1);
            circle(img, armor.right_light.bottom, 5, Scalar(0, 0, 255), -1);
            circle(img, armor.right_light.top, 5, Scalar(0, 0, 255), -1);
        }
    }
    return final_armors;
}
