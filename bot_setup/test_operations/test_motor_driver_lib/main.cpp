#include "motor_driver/motorDriver.hpp"
#include <memory>
#include <iostream>
#include <fmt/core.h>
#include <unistd.h>

//GPIO pins and L298N pins
#define IN1 23
#define IN2 24
#define IN3 17
#define IN4 27
#define ENA 25
#define ENB 22

using motorDriver::MotorDriver;
using motorDriver::MotorDirection;
using namespace fmt;

int main()
{
    std::unique_ptr<MotorDriver> leftMotors = std::make_unique<MotorDriver>(
        IN3, IN4, ENB, "leftMotors"
    );
    std::unique_ptr<MotorDriver> rightMotors = std::make_unique<MotorDriver>(
        IN1, IN2, ENA, "leftMotors"
    );

    print("Moving forward at half effort\n");
    leftMotors->setEffortPercent(50.0);
    rightMotors->setEffortPercent(50.0);
    sleep(3);

    //Stop
    leftMotors->setEffortPercent(0.0);
    rightMotors->setEffortPercent(0.0);
    sleep(1.5);

    print("Moving backward at half effort\n");
    leftMotors->setDirection(MotorDirection::Backward);
    rightMotors->setDirection(MotorDirection::Backward);
    leftMotors->setEffortPercent(50.0);
    rightMotors->setEffortPercent(50.0);
    sleep(3);

    //Stop
    leftMotors->setEffortPercent(0.0);
    rightMotors->setEffortPercent(0.0);
    sleep(1.5);

    print("Moving clockwise\n");
    leftMotors->setDirection(MotorDirection::Forward);
    rightMotors->setDirection(MotorDirection::Backward);
    leftMotors->setEffortPercent(100.0);
    rightMotors->setEffortPercent(100.0);
    sleep(3);

    //Stop
    leftMotors->setEffortPercent(0.0);
    rightMotors->setEffortPercent(0.0);
    sleep(1.5);

    print("Moving counter-clockwise\n");
    leftMotors->setDirection(MotorDirection::Backward);
    rightMotors->setDirection(MotorDirection::Forward);
    leftMotors->setEffortPercent(100.0);
    rightMotors->setEffortPercent(100.0);
    sleep(3.0);

    //Stop
    leftMotors->setEffortPercent(0.0);
    rightMotors->setEffortPercent(0.0);
    sleep(1.5);

    print("Moving forward at full effort\n");
    leftMotors->setDirection(MotorDirection::Forward);
    rightMotors->setDirection(MotorDirection::Forward);
    leftMotors->setEffortPercent(100.0);
    rightMotors->setEffortPercent(100.0);
    sleep(3);

    //Stop
    leftMotors->setEffortPercent(0.0);
    rightMotors->setEffortPercent(0.0);
    sleep(1.5);

    print("Moving backward at full effort\n");
    leftMotors->setDirection(MotorDirection::Backward);
    rightMotors->setDirection(MotorDirection::Backward);
    leftMotors->setEffortPercent(100.0);
    rightMotors->setEffortPercent(100.0);
    sleep(3);

    print("Testing Done.\n");

    return 0;
}