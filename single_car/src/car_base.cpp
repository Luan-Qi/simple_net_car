#include "rclcpp/rclcpp.hpp"
#include "car_base.hpp"
#include <chrono>
#include "wiringPi.h"
#include "softPwm.h"


// class SmartCar{
// public:
//     SmartCar(){
//         if (wiringPiSetup() == -1) {
//             return;
//         }

//         softPwmCreate(MOTOR1_PWMA_PIN, 0, 100);
//         softPwmCreate(MOTOR1_PWMB_PIN, 0, 100);
//         softPwmCreate(MOTOR2_PWMA_PIN, 0, 100);
//         softPwmCreate(MOTOR2_PWMB_PIN, 0, 100);
//         goStop();
//     };

//     SmartCar(const int &FL, const int &FR, const int &BL, const int &BR)
//              : state(0),  FL(FL), FR(FR), BL(BL), BR(BR){
//         wiringPiSetup () ;
//         pinMode (FL, OUTPUT) ;
//         pinMode (FR, OUTPUT) ;
//         pinMode (BL, OUTPUT) ;
//         pinMode (BR, OUTPUT) ;
//         goStop();
//     };
//     //destructor
//     ~SmartCar(){goStop();};

//     void go(const int& d);
//     void goForward(int speed);
//     void goBackward(int speed);
//     void goLeft(int speed);
//     void goRight(int speed);
//     void goBrake();
//     void goStop();
//     void goVector(float axis1, float axis2);
//     int getCurState(){return state;}

// private:
//     int state = 0;
//     int FL = 21;
//     int FR = 22;
//     int BL = 23;
//     int BR = 24;
//     int MOTOR1_PWMA_PIN = 1;
//     int MOTOR1_PWMB_PIN = 2;
//     int MOTOR2_PWMA_PIN = 3;
//     int MOTOR2_PWMB_PIN = 4;
// };

SmartCar::SmartCar(){
    if (wiringPiSetup() == -1) {
        return;
    }

    softPwmCreate(MOTOR1_PWMA_PIN, 0, 100);
    softPwmCreate(MOTOR1_PWMB_PIN, 0, 100);
    softPwmCreate(MOTOR2_PWMA_PIN, 0, 100);
    softPwmCreate(MOTOR2_PWMB_PIN, 0, 100);
    goStop();
};

SmartCar::SmartCar(const int &L1, const int &L2, const int &R1, const int &R2)
        : state(0),  MOTOR1_PWMA_PIN(L1), MOTOR1_PWMB_PIN(L2), MOTOR2_PWMA_PIN(R1), MOTOR2_PWMB_PIN(R2){
    if (wiringPiSetup() == -1) {
        return;
    }

    softPwmCreate(MOTOR1_PWMA_PIN, 0, 100);
    softPwmCreate(MOTOR1_PWMB_PIN, 0, 100);
    softPwmCreate(MOTOR2_PWMA_PIN, 0, 100);
    softPwmCreate(MOTOR2_PWMB_PIN, 0, 100);
    goStop();
};

void SmartCar::goForward(int speed){
    goMotorLeft(speed);
    goMotorRight(speed);
    PWMA1 = speed;PWMA2 = 0;PWMB1 = speed;PWMB2 = 0;
}

void SmartCar::goBackward(int speed){
    goMotorLeft(-speed);
    goMotorRight(-speed);
    PWMA1 = 0;PWMA2 = speed;PWMB1 = 0;PWMB2 = speed;
}

void SmartCar::goLeft(int speed){
    goMotorLeft(-speed);
    goMotorRight(speed);
    PWMA1 = 0;PWMA2 = speed;PWMB1 = speed;PWMB2 = 0;
}

void SmartCar::goRight(int speed){
    goMotorLeft(speed);
    goMotorRight(-speed);
    PWMA1 = speed;PWMA2 = 0;PWMB1 = 0;PWMB2 = speed;
}

void SmartCar::goBrake(){
    softPwmWrite(MOTOR1_PWMA_PIN, 100);
    softPwmWrite(MOTOR1_PWMB_PIN, 100);
    softPwmWrite(MOTOR2_PWMA_PIN, 100);
    softPwmWrite(MOTOR2_PWMB_PIN, 100);
    PWMA1 = 100;PWMA2 = 100;PWMB1 = 100;PWMB2 = 100;
}

void SmartCar::goStop(){
    softPwmWrite(MOTOR1_PWMA_PIN, 0);
    softPwmWrite(MOTOR1_PWMB_PIN, 0);
    softPwmWrite(MOTOR2_PWMA_PIN, 0);
    softPwmWrite(MOTOR2_PWMB_PIN, 0);
    PWMA1 = 0;PWMA2 = 0;PWMB1 = 0;PWMB2 = 0;
}

void SmartCar::goVector(float axis1, float axis2) {
    // 计算基础轮速并叠加转向差速
    float leftSpeed = axis1 - axis2;
    float rightSpeed = axis1 + axis2;

    // 控制左轮
    goMotorLeft(static_cast<int>(leftSpeed * 50));
    // 控制右轮
    goMotorRight(static_cast<int>(rightSpeed * 50));
}

void SmartCar::goMotorLeft(int speed){
    if(speed>=0)
    {
        softPwmWrite(MOTOR1_PWMA_PIN, speed);
        softPwmWrite(MOTOR1_PWMB_PIN, 0);
    }
    else
    {
        softPwmWrite(MOTOR1_PWMA_PIN, 0);
        softPwmWrite(MOTOR1_PWMB_PIN, -speed);
    }
}

void SmartCar::goMotorRight(int speed){
    if(speed>=0)
    {
        softPwmWrite(MOTOR2_PWMA_PIN, speed);
        softPwmWrite(MOTOR2_PWMB_PIN, 0);
    }
    else
    {
        softPwmWrite(MOTOR2_PWMA_PIN, 0);
        softPwmWrite(MOTOR2_PWMB_PIN, -speed);
    }
}