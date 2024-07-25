#ifndef SOCKET_VIDEO_RECEIVER_HPP
#define SOCKET_VIDEO_RECEIVER_HPP

#include <opencv2/opencv.hpp>
#include <string>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

class SocketVideoReceiver {
public:
    SocketVideoReceiver(const std::string& address, int port, const std::string& serverIp, int serverPort);
    ~SocketVideoReceiver();

    void startReceiving();

private:
    std::string serverAddress;
    int serverPort;
    cv::VideoCapture videoCapture;
    int poseSocket;
    struct sockaddr_in serverAddr;

    void sendPose(const cv::Mat& prediction);
};

#endif // SOCKET_VIDEO_RECEIVER_HPP
