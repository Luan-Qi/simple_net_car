#include <rclcpp/rclcpp.hpp>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <thread>
#include <memory>
#include <string>
#include <vector>
#include <regex>
#include <mutex>
#include <fcntl.h>
#include <errno.h>
#include <unordered_set>
#include <chrono>

class SocketServer : public rclcpp::Node {
public:
    SocketServer(const std::string& ip, int port) 
        : Node("socket_server"), ip_(ip), port_(port) {
        this->declare_parameter("reuse_address", true);
        
        // 初始化服务器
        if (!init_server()) {
            RCLCPP_ERROR(this->get_logger(), "Server initialization failed");
            rclcpp::shutdown();
        }
        
        // 启动接受连接线程
        accept_thread_ = std::thread(&SocketServer::accept_connections, this);
    }

    ~SocketServer() {
        shutdown_server();
    }

private:
    void shutdown_server() {
        if (!running_.exchange(false)) return;

        // 关闭服务器socket以中断accept调用
        if (server_fd_ != -1) {
            shutdown(server_fd_, SHUT_RDWR);
            close(server_fd_);
            server_fd_ = -1;
        }

        // 关闭所有客户端连接
        {
            std::lock_guard<std::mutex> lock(clients_mutex_);
            for (int fd : client_fds_) {
                shutdown(fd, SHUT_RDWR);
                close(fd);
            }
            client_fds_.clear();
        }

        // 等待接受线程结束
        if (accept_thread_.joinable()) {
            accept_thread_.join();
        }

        RCLCPP_INFO(this->get_logger(), "Server shutdown complete");
    }

    bool init_server() {
        // 创建socket
        server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
        if (server_fd_ == -1) {
            RCLCPP_ERROR(this->get_logger(), "Socket creation error: %s", strerror(errno));
            return false;
        }

        // 设置socket选项
        int reuse = this->get_parameter("reuse_address").as_bool();
        if (setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Set socket options error: %s", strerror(errno));
            return false;
        }

        // 绑定地址
        sockaddr_in server_addr{};
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(port_);
        inet_pton(AF_INET, ip_.c_str(), &server_addr.sin_addr);

        if (bind(server_fd_, (sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Bind error: %s", strerror(errno));
            return false;
        }

        // 开始监听
        if (listen(server_fd_, 5) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Listen error: %s", strerror(errno));
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "Server started on %s:%d", ip_.c_str(), port_);
        return true;
    }

    void accept_connections() {
        while (rclcpp::ok() && running_) {
            sockaddr_in client_addr{};
            socklen_t client_len = sizeof(client_addr);
            
            int client_fd = accept(server_fd_, 
                                  (sockaddr*)&client_addr, 
                                  &client_len);
            if (client_fd < 0) {
                if (running_) {
                    RCLCPP_ERROR(this->get_logger(), "Accept error: %s", strerror(errno));
                }
                continue;
            }

            // 获取客户端信息
            char client_ip[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, &client_addr.sin_addr, client_ip, INET_ADDRSTRLEN);
            RCLCPP_INFO(this->get_logger(), 
                       "New connection from %s:%d", 
                       client_ip, 
                       ntohs(client_addr.sin_port));
            
            // 记录客户端描述符
            std::lock_guard<std::mutex> lock(clients_mutex_);
            client_fds_.insert(client_fd);

            // 创建客户端线程
            std::thread([this, client_fd, client_ip]() {
                handle_client(client_fd, client_ip);
            }).detach();
        }
    }

    void handle_client(int fd, const std::string& client_ip) {
        // 设置非阻塞模式避免recv卡死
        //set_nonblocking(fd);

        constexpr size_t BUFFER_SIZE = 1024;
        char buffer[BUFFER_SIZE];

        // 在while循环开始前初始化last_loop_time_
        //last_loop_time_ = std::chrono::steady_clock::now();

        while (running_) {
            if (!running_) break;

            // // 计算当前循环迭代的时间间隔
            // auto current_time = std::chrono::steady_clock::now();
            // auto time_elapsed = std::chrono::duration_cast<std::chrono::microseconds>(current_time - last_loop_time_);
            // last_loop_time_ = current_time; // 更新last_loop_time_为当前时间

            // // 计算频率（Hz）
            // double frequency = 1.0 / (time_elapsed.count() * 0.000001);

            // // 打印频率
            // RCLCPP_INFO(this->get_logger(), "Loop frequency: %.2f Hz", frequency);

            ssize_t bytes_received = recv(fd, buffer, BUFFER_SIZE - 1, 0);
            
            if (bytes_received <= 0) {
                if (bytes_received == 0) {
                    RCLCPP_INFO(this->get_logger(), 
                               "Client %s disconnected", client_ip.c_str());
                } else {
                    RCLCPP_ERROR(this->get_logger(), 
                               "Receive error from %s: %s", 
                               client_ip.c_str(), 
                               strerror(errno));
                }
                break;
            }

            // 处理接收到的数据
            buffer[bytes_received] = '\0';
            RCLCPP_INFO(this->get_logger(), 
                      "Received from %s: %s", 
                      client_ip.c_str(), 
                      buffer);

            // 发送响应
            std::string response = "Server received: ";
            response += buffer;
            if (send(fd, response.c_str(), response.size(), 0) < 0) {
                RCLCPP_ERROR(this->get_logger(), 
                           "Send error to %s: %s", 
                           client_ip.c_str(), 
                           strerror(errno));
                break;
            }
        }

        std::lock_guard<std::mutex> lock(clients_mutex_);
        client_fds_.erase(fd);

        close(fd);
    }


    void set_nonblocking(int fd) {
        int flags = fcntl(fd, F_GETFL, 0);   // 获取当前文件状态标志
        if (flags == -1) {
            RCLCPP_ERROR(this->get_logger(), "fcntl(F_GETFL) error: %s", strerror(errno));
            return;
        }
        if (fcntl(fd, F_SETFL, flags | O_NONBLOCK) == -1) {  // 设置非阻塞模式
            RCLCPP_ERROR(this->get_logger(), "fcntl(F_SETFL) error: %s", strerror(errno));
        }
    }


    int server_fd_;
    std::thread accept_thread_;
    std::string ip_;
    int port_;
    std::atomic<bool> running_{true};
    std::mutex clients_mutex_;
    std::unordered_set<int> client_fds_;

    // 在类的成员变量中添加一个时间点变量
    std::chrono::time_point<std::chrono::steady_clock> last_loop_time_;
};

bool isValidIP(const std::string& ip) {
    // 使用正则表达式验证 IP 地址格式
    std::regex ip_regex("^((25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\\.){3}(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)$");
    return std::regex_match(ip, ip_regex);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    // 可配置参数
    std::string server_ip = "127.0.0.1";  // 监听所有接口
    int server_port = 12345;  // 监听端口

        // 解析命令行参数
    if (argc > 1) {
        server_ip = argv[1];
        if (!isValidIP(server_ip)) {
            std::cerr << "Invalid argument for server IP: " << server_ip << " is not a valid IP address." << std::endl;
            return 1;
        }
    }
    if (argc > 2) {
        try {
            server_port = std::stoi(argv[2]);
        } catch (const std::invalid_argument& ia) {
            std::cerr << "Invalid argument for server port: " << argv[2] << " is not a valid integer." << std::endl;
            return 1;
        } catch (const std::out_of_range& oor) {
            std::cerr << "Out of range error for server port: " << argv[2] << " is out of range." << std::endl;
            return 1;
        }
    }
    
    auto server = std::make_shared<SocketServer>(server_ip, server_port);
    rclcpp::spin(server);
    rclcpp::shutdown();
    return 0;
}

