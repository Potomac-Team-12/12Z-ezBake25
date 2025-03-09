#pragma once

#include "main.h"

extern pros::Controller master;

extern pros::ADIDigitalOut doinker;
extern pros::ADIDigitalOut goalClamp1;
extern pros::ADIDigitalOut goalClamp2;
extern pros::ADIDigitalOut ClampLED;

extern pros::Rotation armSensor;
extern pros::Motor armMotor;

extern pros::Motor intake1;
extern pros::Motor intake2;

extern pros::MotorGroup left_side_motors;
extern pros::MotorGroup right_side_motors;


extern bool wingsOut;
extern bool liftUp;
extern bool goalClampToggle;

extern Drive chassis;

void checkMotorsAndPrintTemperature();
void calibrateChassis();
void chassisInits();
