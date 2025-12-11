#include "main.h"
#include "lemlib/api.hpp"

using namespace pros;

extern Motor frontLeft;
extern Motor midLeft;
extern Motor backLeft;
extern Motor frontRight;
extern Motor midRight;
extern Motor backRight;

extern pros::ADILed led;

extern Controller master;

extern MotorGroup leftMotors;
extern MotorGroup rightMotors;

extern Imu imuTop;
extern Imu imuBottom;

extern Rotation vertical_Tracking;
extern Rotation horizontal_Tracking;

extern pros::Controller master;

extern lemlib::Drivetrain drivetrain;
extern lemlib::OdomSensors odomInfo;

extern lemlib::Chassis chassis;