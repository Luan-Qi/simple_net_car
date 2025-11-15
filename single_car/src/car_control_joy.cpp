#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "car_base.hpp"

class CarControlNode : public rclcpp::Node
{
public:
    CarControlNode() : Node("car_control_node")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&CarControlNode::joy_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Joy_control node started!");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    SmartCar smart_car_;

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        if(msg->buttons[0]>0.5) 
        {
            smart_car_.goBrake();
            return;
        }
        else if(msg->axes[2]<0.999 || msg->axes[5]<0.999) 
        {
            float speed = msg->axes[2]-msg->axes[5];
            if(speed<=0){smart_car_.goBrake();}
            else{smart_car_.goForward(static_cast<int>(speed*50));}
            return;
        }
        else if(msg->buttons[4]>0.5 || msg->buttons[5]>0.5)
        {
            smart_car_.goMotorLeft(static_cast<int>(msg->buttons[4]*50));
            smart_car_.goMotorRight(static_cast<int>(msg->buttons[5]*50));
            return;
        }
        else if(msg->axes[6]!=0 || msg->axes[7]!=0) 
        {
            int state1 = (msg->axes[6] == 1) ? 1 : 0;
            int state2 = (msg->axes[6] == -1) ? 1 : 0;
            int state3 = (msg->axes[7] == 1) ? 1 : 0;
            int state4 = (msg->axes[7] == -1) ? 1 : 0;
            int combinedState = (state4 << 3) | (state3 << 2) | (state2 << 1) | state1;

            // 使用switch语句处理组合状态
            switch (combinedState) {
                case 1:  // 0001
                    smart_car_.goLeft(50);
                    //std::cout << "goLeft" << std::endl;
                    break;
                case 2: // 0010
                    smart_car_.goRight(50);
                    //std::cout << "goRight" << std::endl;
                    break;
                case 4: // 0100
                    smart_car_.goForward(50);
                    //std::cout << "goForward" << std::endl;
                    break;
                case 5: // 0101
                    smart_car_.goMotorLeft(30);
                    smart_car_.goMotorRight(60);
                    //std::cout << "goForwardLeft" << std::endl;
                    break;
                case 6: // 0110
                    smart_car_.goMotorLeft(60);
                    smart_car_.goMotorRight(30);
                    //std::cout << "goForwardRight" << std::endl;
                    break;
                case 8: // 1000
                    smart_car_.goBackward(50);
                    //std::cout << "goBackward" << std::endl;
                    break;
                case 9: // 1001
                    smart_car_.goMotorLeft(-30);
                    smart_car_.goMotorRight(-60);
                    //std::cout << "goBackwardLeft" << std::endl;
                    break;
                case 10: // 1010
                    smart_car_.goMotorLeft(-60);
                    smart_car_.goMotorRight(-30);
                    //std::cout << "goBackwardRight" << std::endl;
                    break;
                default:
                    smart_car_.goStop();
                    //std::cout << "goStop" << std::endl;
                    break;
            }
            return;
        }
        else smart_car_.goVector(0.6*msg->axes[1], 0.5*msg->axes[0]);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CarControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}



