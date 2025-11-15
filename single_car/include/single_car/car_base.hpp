#ifndef CAR_BASE_HPP
#define CAR_BASE_HPP

#include "rclcpp/rclcpp.hpp"

class SmartCar {
public:
    SmartCar();
    SmartCar(const int &FL, const int &FR, const int &BL, const int &BR);
    ~SmartCar(){goStop();};

    void goForward(int speed);
    void goBackward(int speed);
    void goLeft(int speed);
    void goRight(int speed);
    void goBrake();
    void goStop();
    void goVector(float axis1, float axis2);
    void goMotorLeft(int speed);
    void goMotorRight(int speed);
    void goSetBoost();
    int getCurState();

private:
    rclcpp::Node::SharedPtr node_handle_; // 假设需要一个 ROS 节点句柄
    int state = 0;
    int PWMA1 = 0;
    int PWMA2 = 0;
    int PWMB1 = 0;
    int PWMB2 = 0;
    int MOTOR1_PWMA_PIN = 1;
    int MOTOR1_PWMB_PIN = 4;
    int MOTOR2_PWMA_PIN = 5;
    int MOTOR2_PWMB_PIN = 6;

    // 其他成员函数声明
};

#endif // CAR_BASE_HPP