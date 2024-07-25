#include "../include/position.hpp"
#include "../include/detector.hpp"
#include "../include/number_classifier.hpp"

Quaternion::Quaternion(double w, double x, double y, double z)
    : w(w), x(x), y(y), z(z) {}

Quaternion Quaternion::operator*(const Quaternion& q) const {
    return Quaternion(
        w * q.w - x * q.x - y * q.y - z * q.z,
        w * q.x + x * q.w + y * q.z - z * q.y,
        w * q.y - x * q.z + y * q.w + z * q.x,
        w * q.z + x * q.y - y * q.x + z * q.w
    );
}

Quaternion Quaternion::conjugate() const {
    return Quaternion(w, -x, -y, -z);
}

Quaternion Quaternion::fromEuler(double roll, double pitch, double yaw) {
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    return Quaternion(
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy
    );
}

void Quaternion::toEulerAngles(double& roll, double& pitch, double& yaw) const {
    double sinr_cosp = 2 * (w * x + y * z);
    double cosr_cosp = 1 - 2 * (x * x + y * y);
    roll = atan2(sinr_cosp, cosr_cosp);

    double sinp = 2 * (w * y - z * x);
    if (fabs(sinp) >= 1)
        pitch = copysign(M_PI / 2, sinp);
    else
        pitch = asin(sinp);

    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    yaw = atan2(siny_cosp, cosy_cosp);
}

void Quaternion::rotatePoint(double& px, double& py, double& pz) const {
    Quaternion p(0, px, py, pz);
    Quaternion conj(w, -x, -y, -z);
    Quaternion result = (*this) * p * conj;
    px = result.x;
    py = result.y;
    pz = result.z;
}

Pose::Pose(double x, double y, double z, double roll, double pitch, double yaw)
    : x(x), y(y), z(z), orientation(Quaternion::fromEuler(roll, pitch, yaw)) {}

Pose::Pose(double x, double y, double z, const Quaternion& orientation)
    : x(x), y(y), z(z), orientation(orientation) {}

Pose Pose::transform(const Quaternion& rotation) const {
    Quaternion new_orientation = orientation * rotation;
    double new_x = x, new_y = y, new_z = z;
    rotation.rotatePoint(new_x, new_y, new_z);

    return Pose(new_x, new_y, new_z, new_orientation);
}

void Pose::print() const {
    std::cout << "Position: (" << x << ", " << y << ", " << z << "), Orientation: " << orientation << std::endl;
}

std::ostream& operator<<(std::ostream& os, const Quaternion& q) {
    os << "(" << q.w << ", " << q.x << ", " << q.y << ", " << q.z << ")";
    return os;
}

Pose trans_final(const std::vector<Armor>& armors) {
    if (armors.empty()) {
        // 返回一个特殊的 Pose 对象，表示没有识别到装甲板
        return Pose(0, 0, 0, 0, 0, 0);
    }
    double x = armors[0].tvec.at<double>(0, 0);  //获取 x 坐标
    double y = armors[0].tvec.at<double>(1, 0);  // 获取 y 坐标
    double z = armors[0].tvec.at<double>(2, 0);  // 获取 z 坐标
    Mat rotation_matrix;
    Rodrigues(armors[0].rvec, rotation_matrix);
    double roll;
    double pitch;
    double yaw;
    pitch = atan2(-rotation_matrix.at<double>(2, 0), sqrt(pow(rotation_matrix.at<double>(2, 1), 2) + pow(rotation_matrix.at<double>(2, 2), 2)));
    yaw = atan2(rotation_matrix.at<double>(1, 0), rotation_matrix.at<double>(0, 0));
    roll = atan2(rotation_matrix.at<double>(2, 1), rotation_matrix.at<double>(2, 2));
    Pose pose(x,y,z,roll,pitch,yaw);
    Quaternion rotation = Quaternion::fromEuler(-M_PI_2, 0, -M_PI_2);
    Pose gimbal_pose = pose.transform(rotation);
    gimbal_pose.x += 0.2;
    return gimbal_pose;
}

void draw_prediction(cv::Mat prediction, cv::Mat& img) {
    Mat cameraMatrix = (Mat_<double>(3, 3) << 2102.080562187802, 0.0, 689.2057889332623, 
                                              0.0, 2094.0179120166754, 529.83878, 
                                              0.0, 0.0, 1.0);
    Mat distCoeffs = (Mat_<double>(1, 5) << -0.06478109387525666, 0.39036067923005396, 
                                             -0.0042514793151166306, 0.008306749648029776, 
                                             -1.6613800909405605);

    // 将预测的位姿转换成Pose对象
    Pose pose(prediction.at<float>(0), prediction.at<float>(1), prediction.at<float>(2),
              prediction.at<float>(3), prediction.at<float>(4), prediction.at<float>(5));
    pose.x -= 0.2;
    // 定义一个四元数进行旋转
    Quaternion rotation = Quaternion::fromEuler(M_PI_2, -M_PI_2, 0);
    Pose camera_pose = pose.transform(rotation);
    // 将四元数转换为旋转矩阵
    cv::Mat rotation_matrix = (cv::Mat_<double>(3, 3) << 
        1 - 2 * (camera_pose.orientation.y * camera_pose.orientation.y + camera_pose.orientation.z * camera_pose.orientation.z),
        2 * (camera_pose.orientation.x * camera_pose.orientation.y - camera_pose.orientation.w * camera_pose.orientation.z),
        2 * (camera_pose.orientation.x * camera_pose.orientation.z + camera_pose.orientation.w * camera_pose.orientation.y),
        2 * (camera_pose.orientation.x * camera_pose.orientation.y + camera_pose.orientation.w * camera_pose.orientation.z),
        1 - 2 * (camera_pose.orientation.x * camera_pose.orientation.x + camera_pose.orientation.z * camera_pose.orientation.z),
        2 * (camera_pose.orientation.y * camera_pose.orientation.z - camera_pose.orientation.w * camera_pose.orientation.x),
        2 * (camera_pose.orientation.x * camera_pose.orientation.z - camera_pose.orientation.w * camera_pose.orientation.y),
        2 * (camera_pose.orientation.y * camera_pose.orientation.z + camera_pose.orientation.w * camera_pose.orientation.x),
        1 - 2 * (camera_pose.orientation.x * camera_pose.orientation.x + camera_pose.orientation.y * camera_pose.orientation.y)
    );

    // 将旋转矩阵转换为旋转向量
    cv::Mat rvec;
    cv::Rodrigues(rotation_matrix, rvec);

    // 获取平移向量
    cv::Mat tvec = (cv::Mat_<double>(3, 1) << camera_pose.x, camera_pose.y, camera_pose.z);

    // 定义3D点
    std::vector<cv::Point3f> Points3d;
    Points3d.push_back(cv::Point3f(0, 0, 0));
    Points3d.push_back(cv::Point3f(0, 5.5, 0));
    Points3d.push_back(cv::Point3f(13.5, 5.5, 0));
    Points3d.push_back(cv::Point3f(13.5, 0, 0));

    // 投影3D点到2D图像平面
    std::vector<cv::Point2f> Points2d;
    cv::projectPoints(Points3d, rvec, tvec, cameraMatrix, distCoeffs, Points2d);

    // 计算中心点
    cv::Point2f center(0, 0);
    for (size_t i = 0; i < Points2d.size(); i++) {
        center += Points2d[i];
    }
    center *= (1.0 / Points2d.size());

    // 绘制中心点
    cv::circle(img, center, 5, cv::Scalar(0, 255, 0), -1);
}


