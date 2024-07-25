#include <iostream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>

#define PORT 54321

int main() {
    int server_fd, new_socket;
    struct sockaddr_in address;
    int addrlen = sizeof(address);
    char buffer[1024] = {0};

    // 创建套接字
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
        std::cerr << "Socket failed" << std::endl;
        exit(EXIT_FAILURE);
    }

    // 绑定地址和端口
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(PORT);

    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
        std::cerr << "Bind failed" << std::endl;
        close(server_fd);
        exit(EXIT_FAILURE);
    }

    // 监听连接
    if (listen(server_fd, 3) < 0) {
        std::cerr << "Listen failed" << std::endl;
        close(server_fd);
        exit(EXIT_FAILURE);
    }

    std::cout << "Server is listening on port " << PORT << std::endl;

    while (true) {
        // 接受连接
        if ((new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t*)&addrlen)) < 0) {
            std::cerr << "Accept failed" << std::endl;
            close(server_fd);
            exit(EXIT_FAILURE);
        }

        std::cout << "Connection established" << std::endl;

        // 不断接收数据直到客户端关闭连接
        while (true) {
            int valread = read(new_socket, buffer, sizeof(buffer) - 1);
            if (valread < 0) {
                std::cerr << "Read failed" << std::endl;
                break;
            } else if (valread == 0) {
                std::cout << "Client disconnected" << std::endl;
                break;
            }
            buffer[valread] = '\0'; // 确保缓冲区以null终止
            std::cout << "Received: " << buffer << std::endl;
        }

        // 关闭当前连接
        close(new_socket);
    }

    // 关闭服务器套接字
    close(server_fd);
    return 0;
}
