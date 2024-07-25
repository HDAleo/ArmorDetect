#include <iostream>
#include <opencv2/opencv.hpp>
#include <detector.hpp>
#include <kalman.hpp>
#include <number_classifier.hpp>
#include <position.hpp>
#include <client_server.hpp>

using namespace std;
using namespace cv;
using namespace armor;

int main() {
    const std::string serverAddress = "/home/hda/code/BigHomeWork/data/test.mp4";
    const int videoStreamPort = 12345;

    const std::string serverIp = "127.0.0.1";
    const int serverPort = 54321;
    
    SocketVideoReceiver receiver(serverAddress, videoStreamPort, serverIp, serverPort);
    receiver.startReceiving();
    return 0;
}