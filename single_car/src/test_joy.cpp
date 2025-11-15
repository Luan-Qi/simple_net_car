#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

class JoySubscriber : public rclcpp::Node {
public:
    JoySubscriber() : Node("joy_subscriber") {
        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&JoySubscriber::joyCallback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Joy subscriber has started.");
    }

private:
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        // 处理摇杆数据
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
            "Axes: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f], Buttons: [%d, %d, %d, %d]", 
            msg->axes[0], msg->axes[1], msg->axes[2], msg->axes[3], msg->axes[4], msg->axes[5], msg->axes[6], msg->axes[7], 
            msg->buttons[0], msg->buttons[1], msg->buttons[2], msg->buttons[3]);
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JoySubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}