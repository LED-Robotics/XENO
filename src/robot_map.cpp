
#include "robot_map.h"

using namespace AngularPID;
using namespace LateralPID;

// Motors

MotorGroup leftMotors({-DrivePorts::kFrontLeft, -DrivePorts::kBackLeft, DrivePorts::kMiddleLeft}, MotorGears::blue, MotorUnits::deg);
MotorGroup rightMotors({DrivePorts::kFrontRight, DrivePorts::kBackRight, -DrivePorts::kMiddleRight}, MotorGears::blue, MotorUnits::deg);

pros::Motor preRollers(-2, MotorGears::rpm_200, MotorUnits::deg);
pros::Motor l2Motor(-1, MotorGears::rpm_200, MotorUnits::deg);

pros::Controller master(CONTROLLER_MASTER);

// Pistons

pros::ADIDigitalOut hood_Piston('C');
pros::ADIDigitalOut dp_Piston('B');
pros::ADIDigitalOut lw_Piston('A');

pros::ADILed led('H', 100);

// Sensors

pros::Distance backRightDistance(8);
pros::Distance frontRightDistance(9);
pros::Distance backLeftDistance(4);
pros::Distance frontLeftDistance(3);
pros::Distance backDistance(10);

pros::Optical colorSort(19);

Imu imuTop{17};

pros::Rotation vertical_Tracking(14);
pros::Rotation horizontal_Tracking(7);

lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_Tracking, lemlib::Omniwheel::NEW_2, -5.75);
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_Tracking, lemlib::Omniwheel::NEW_2, -5.75);

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
    nullptr, // set both to nullptr because we dont have vertical tracking wheels
    nullptr,
    nullptr, // horizontal tracking wheel 1
    nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
    &imuTop, // inertial sensor

};

// lateral PID controller
lemlib::ControllerSettings lateral_controller(
    LateralPID::lateralkP,                     // proportional gain (kP)
    LateralPID::lateralkI,                     // integral gain (kI)
    LateralPID::lateralkD,                     // derivative gain (kD)
    LateralPID::lateralAntiWindup,             // anti windup
    LateralPID::lateralSmallErrorRange,        // small error range, in inches
    LateralPID::lateralSmallErrorRangeTimeout, // small error range timeout, in milliseconds
    LateralPID::lateralLargeErrorRange,        // large error range, in inches
    LateralPID::lateralLargeErrorRangeTimeout, // large error range timeout, in milliseconds
    LateralPID::lateralSlew                    // maximum acceleration (slew)
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
