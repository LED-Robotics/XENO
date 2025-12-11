#include "main.h"
#include "robot_map.h"
#include "constants.h"

// Motor frontLeft{DrivePorts::kFrontLeft, MotorGears::blue, MotorUnits::deg};
// Motor frontRight{DrivePorts::kFrontRight, MotorGears::blue, MotorUnits::deg};
// Motor backLeft{DrivePorts::kBackLeft, MotorGears::blue, MotorUnits::deg};
// Motor backRight{DrivePorts::kBackRight, MotorGears::blue, MotorUnits::deg};

using namespace AngularPID;
using namespace LateralPID;

// MotorGroup leftMotors({frontLeft, backLeft});
// MotorGroup rightMotors({frontRight, backRight});
MotorGroup leftMotors({DrivePorts::kFrontLeft, DrivePorts::kBackLeft, DrivePorts::kMiddleLeft}, MotorGears::blue, MotorUnits::deg);
MotorGroup rightMotors({DrivePorts::kFrontRight, DrivePorts::kBackRight, DrivePorts::kMiddleRight}, MotorGears::blue, MotorUnits::deg);

pros::ADILed led('H', 100);

pros::Controller master(CONTROLLER_MASTER);

Imu imuTop{8};
Imu imuBottom{9};

pros::Rotation vertical_Tracking(1);
pros::Rotation horizontal_Tracking(2);
// create a v5 rotation sensor on port 1

lemlib::Drivetrain drivetrain{
    //>OWO<
    &leftMotors,
    &rightMotors,
    OdometryInfo::kTrackWidth,
    OdometryInfo::kWheelDiameter,
    OdometryInfo::kWheelRPM,
    OdometryInfo::kDriftConstant // 2.00 for now
};

lemlib::OdomSensors odomInfo{
    vertical_Tracking, // set both to nullptr because we dont have vertical tracking wheels
    nullptr,
    horizontal_Tracking, // horizontal tracking wheel 1
    nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
    &imuTop, // inertial sensor

};

// lateral PID controller
lemlib::ControllerSettings lateral_controller(
    lateralkP,                     // proportional gain (kP)
    lateralkI,                     // integral gain (kI)
    lateralkD,                     // derivative gain (kD)
    lateralAntiWindup,             // anti windup
    lateralSmallErrorRange,        // small error range, in inches
    lateralSmallErrorRangeTimeout, // small error range timeout, in milliseconds
    lateralLargeErrorRange,        // large error range, in inches
    lateralLargeErrorRangeTimeout, // large error range timeout, in milliseconds
    lateralSlew                    // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(
    angularkP,                     // proportional gain (kP)
    angularkI,                     // integral gain (kI)
    angularkD,                     // derivative gain (kD)
    angularAntiWindup,             // anti windup
    angularSmallErrorRange,        // small error range, in degrees
    angularSmallErrorRangeTimeout, // small error range timeout, in milliseconds
    angularLargeErrorRange,        // large error range, in degrees
    angularLargeErrorRangeTimeout, // large error range timeout, in milliseconds
    angularSlew                    // maximum acceleration (slew)
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttle_curve(3,    // joystick deadband out of 127
                                      10,   // minimum output where drivetrain will move out of 127
                                      1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steer_curve(3,    // joystick deadband out of 127
                                   10,   // minimum output where drivetrain will move out of 127
                                   1.019 // expo curve gain
);

lemlib::Chassis chassis{
    drivetrain,
    lateral_controller,
    angular_controller,
    odomInfo};
