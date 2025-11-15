#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

class CameraPublisher : public rclcpp::Node
{
public:
    CameraPublisher() : Node("camera_publisher")
    {
        // 初始化摄像头（推荐使用OpenCV接口）
        cap.open(0);  // 0表示第一个摄像头设备
        if (!cap.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "无法打开摄像头");
            rclcpp::shutdown();
        }

        // 设置摄像头参数
        cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
        cap.set(cv::CAP_PROP_FPS, 30);

        // 创建图像发布者
        publisher_ = image_transport::create_publisher(this, "camera/image");
    }

    void capture_and_publish() {
        cv::Mat frame;
        if (cap.read(frame)) {
            auto msg = cv_bridge::CvImage(
                std_msgs::msg::Header(), 
                "bgr8", 
                frame
            ).toImageMsg();

            msg->header.stamp = this->now();
            msg->header.frame_id = "camera_frame";

            // 直接发布图像（无需 CameraInfo）
            publisher_.publish(msg); // ✅ 参数正确
        }
    }

private:
    cv::VideoCapture cap;
    image_transport::Publisher publisher_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraPublisher>();
    rclcpp::WallRate loop_rate(30);  // 30Hz

    while (rclcpp::ok()) {
        node->capture_and_publish();
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}