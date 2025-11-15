#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <signal.h>
#include <fcntl.h>
#include <ifaddrs.h>
#include <arpa/inet.h>

#define PORT 12321
#define BUFFER_SIZE 4096


void print_local_ip_addresses() {
    struct ifaddrs *ifaddr, *ifa;
    
    if (getifaddrs(&ifaddr)) {
        perror("获取网络接口信息失败");
        return;
    }

    std::cout << "服务器可用IP地址列表：" << std::endl;
    
    // 遍历所有网络接口
    for (ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
        if (!ifa->ifa_addr) continue;

        // 只显示IPv4地址
        if (ifa->ifa_addr->sa_family == AF_INET) {
            struct sockaddr_in *addr = (struct sockaddr_in *)ifa->ifa_addr;
            char ip[INET_ADDRSTRLEN];
            
            // 转换二进制地址为字符串格式
            inet_ntop(AF_INET, &addr->sin_addr, ip, sizeof(ip));
            
            // 排除回环地址（127.0.0.1）
            if (strcmp(ip, "127.0.0.1") != 0) {
                std::cout << " - " << ifa->ifa_name << ": " << ip << std::endl;
            }
        }
    }

    freeifaddrs(ifaddr);
}

int main() {
    // 忽略 SIGPIPE 信号，防止 send 失败后崩溃
    signal(SIGPIPE, SIG_IGN);

    // 创建 TCP socket
    int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd == -1) {
        perror("socket failed");
        return 1;
    }

    // 绑定端口
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(PORT);

    if (bind(server_fd, (struct sockaddr*)&address, sizeof(address)) < 0) {
        perror("bind failed");
        return 1;
    }

    if (listen(server_fd, 5) < 0) {
        perror("listen failed");
        return 1;
    }

    // 测试摄像头设备
    int camera_fd = open("/dev/video0", O_RDWR);
    if (camera_fd == -1) {
        perror("camera open failed");
        return 1;
    }
    close(camera_fd);

    std::cout << "\n服务器已启动，监听端口 " << PORT << std::endl;
    print_local_ip_addresses();
    std::cout << "等待客户端连接...\n";

    while (true) {
        int client_fd = accept(server_fd, nullptr, nullptr);
        if (client_fd < 0) {
            perror("accept failed");
            continue;
        }

        std::cout << "\n客户端已连接，启动 ffmpeg...\n";

        // 启动 ffmpeg 采集摄像头并编码为 h264（硬件加速），输出为 MPEG-TS 流
        const char* cmd =
            "ffmpeg -f v4l2 -framerate 30 -video_size 640x480 "
            "-pixel_format yuyv422 -i /dev/video0 "
            "-c:v h264_v4l2m2m -g 10 -bf 0 -tune zerolatency "
            "-b:v 4M -f mpegts -";

        FILE* ffmpeg_pipe = popen(cmd, "r");
        if (!ffmpeg_pipe) {
            perror("popen failed");
            close(client_fd);
            continue;
        }

        char buffer[BUFFER_SIZE];
        size_t n;

        // 不停读取 ffmpeg 输出并通过 socket 发送给客户端
        while ((n = fread(buffer, 1, sizeof(buffer), ffmpeg_pipe)) > 0) {
            ssize_t sent = send(client_fd, buffer, n, 0);
            if (sent < 0) {
                perror("send failed（客户端断开？）");
                break;
            }
        }

        std::cout << "ffmpeg 结束，关闭当前客户端连接，继续监听...\n";
        pclose(ffmpeg_pipe);
        close(client_fd);
    }

    close(server_fd);
    return 0;
}