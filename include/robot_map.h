#pragma once

#include "main.h"
#include "lemlib/api.hpp"
#include "constants.h"

using namespace pros;

// Motors

extern Motor frontLeft;
extern Motor midLeft;
extern Motor backLeft;
extern Motor frontRight;
extern Motor midRight;
extern Motor backRight;

extern Motor preRollers;

extern Motor storageMotor;
extern Motor l2Motor;
extern Motor l3Motor;

extern MotorGroup leftMotors;
extern MotorGroup rightMotors;

extern pros::Controller master;

// Sensors

extern pros::ADILed led;

extern Controller master;

extern Distance backRightDistance;
extern Distance frontRightDistance;
extern Distance backLeftDistance;
extern Distance frontLeftDistance;

extern Optical colorSort;

extern Distance intakeChecker;

extern Imu imuTop;

extern Rotation vertical_Tracking;
extern Rotation horizontal_Tracking;

// Pistons

extern pros::ADIDigitalOut hood_Piston;
extern pros::ADIDigitalOut dp_Piston;
extern pros::ADIDigitalOut lw_Piston;

// Lemlib

extern lemlib::Drivetrain drivetrain;
extern lemlib::OdomSensors odomInfo;

extern lemlib::ControllerSettings angular_controller;
extern lemlib::ControllerSettings lateral_controller;

extern lemlib::Chassis chassis;