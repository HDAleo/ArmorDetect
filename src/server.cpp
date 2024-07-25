#include "../include/client_server.hpp"
#include "../include/detector.hpp"
#include "../include/kalman.hpp"
#include "../include/number_classifier.hpp"
#include "../include/position.hpp"
#include <chrono>

using namespace std;
using namespace cv;
using namespace armor;
using namespace std::chrono;

SocketVideoReceiver::SocketVideoReceiver(const std::string& address, int port, const std::string& serverIp, int serverPort)
    : serverAddress(address), serverPort(port) {
    // Initialize socket for sending pose
    poseSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (poseSocket < 0) {
        std::cerr << "Error: Could not create socket" << std::endl;
        exit(EXIT_FAILURE);
    }

    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(serverPort);
    if (inet_pton(AF_INET, serverIp.c_str(), &serverAddr.sin_addr) <= 0) {
        std::cerr << "Invalid address/ Address not supported" << std::endl;
        exit(EXIT_FAILURE);
    }

    if (connect(poseSocket, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) < 0) {
        std::cerr << "Connection Failed" << std::endl;
        exit(EXIT_FAILURE);
    }
}

SocketVideoReceiver::~SocketVideoReceiver() {
    if (videoCapture.isOpened()) {
        videoCapture.release();
    }
    close(poseSocket);
}

void SocketVideoReceiver::startReceiving() {
    videoCapture.open(serverAddress);

    if (!videoCapture.isOpened()) {
        std::cerr << "Error: Could not open video stream from " << serverAddress << std::endl;
        return;
    }
    bool flag = true;
    cv::Mat frame;
    Detect detect;
    vector<Armor> armors;
    KalmanFilter3D kf;
    int frameCount = 0;
    auto startTime = high_resolution_clock::now(); // 开始时间

    while (true) {
        videoCapture >> frame;
        if (frame.empty()) break;
        cv::Mat prediction = cv::Mat::zeros(1, 6, CV_32F);
        armors = detect.detectArmor(frame);
        Pose pose = trans_final(armors);
        if (flag) {
            kf.init(pose);
            flag = false;
        } else if ((pose.x != 0 || pose.y != 0 || pose.z != 0) && !flag) {
            prediction = kf.updateAndPredict(pose, frame);
        }

        frameCount++;
        auto currentTime = high_resolution_clock::now();
        auto duration = duration_cast<seconds>(currentTime - startTime).count();
        double fps = frameCount / (duration + 1e-9); // 计算fps，避免除以0

        // 在视频帧上显示fps
        putText(frame, "FPS: " + to_string(fps), Point(10, 30), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 0), 2);

        imshow("Display Image", frame);
        sendPose(prediction);

        char key = waitKey(1); // 这里修改为等待1毫秒，以实现视频流播放效果
        if (key == 27)
            break;
    }
    videoCapture.release();
    destroyAllWindows();
}

void SocketVideoReceiver::sendPose(const cv::Mat& prediction) {
    if (prediction.at<float>(0) + prediction.at<float>(1) + prediction.at<float>(2) +
        prediction.at<float>(3) + prediction.at<float>(4) + prediction.at<float>(5) != 0) {
        char buffer[1024];
        std::snprintf(buffer, sizeof(buffer), "Pose: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
                      prediction.at<float>(0), prediction.at<float>(1), prediction.at<float>(2),
                      prediction.at<float>(3), prediction.at<float>(4), prediction.at<float>(5));
        send(poseSocket, buffer, std::strlen(buffer), 0);
    }
}
