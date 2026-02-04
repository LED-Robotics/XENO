#pragma once

#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep

namespace DrivePorts
{
    constexpr int kFrontLeft = -8;
    constexpr int kFrontRight = -3;
    constexpr int kBackLeft = 9;
    constexpr int kBackRight = -2;
    constexpr int kMiddleRight = 1;
    constexpr int kMiddleLeft = -10;

}

namespace OdometryInfo
{
    constexpr double kTrackWidth = 12;
    constexpr float kWheelDiameter = lemlib::Omniwheel::OLD_325; // Fix classes
    constexpr double kWheelRPM = 450.0;
    constexpr double kMainGearRatio = 5 / 3;
    constexpr double kDriftConstant = 2.0;

}

namespace LateralPID
{                                                         // need to change
    constexpr double lateralkP = 6.0;                     // proportional gain
    constexpr double lateralkI = 0.0;                     // integral gain
    constexpr double lateralkD = 10.0;                    // derivative gain
    constexpr double lateralAntiWindup = 0.0;             // anti windup
    constexpr double lateralSmallErrorRange = 0.0;        // degrees
    constexpr double lateralSmallErrorRangeTimeout = 0.0; // milliseconds
    constexpr double lateralLargeErrorRange = 0.0;        // degrees
    constexpr double lateralLargeErrorRangeTimeout = 0.0; // milliseconds
    constexpr double lateralSlew = 50.0;                  // maximum acceleration

}

namespace AngularPID
{
    constexpr double angularkP = 1.8;                     // proportional gain
    constexpr double angularkI = 0.0;                     // integral gain
    constexpr double angularkD = 12.25;                   // derivative gain
    constexpr double angularAntiWindup = 0.0;             // anti windup
    constexpr double angularSmallErrorRange = 0.0;        // degrees
    constexpr double angularSmallErrorRangeTimeout = 0.0; // milliseconds
    constexpr double angularLargeErrorRange = 0.0;        // degrees
    constexpr double angularLargeErrorRangeTimeout = 0.0; // milliseconds
    constexpr double angularSlew = 0.0;                   // maximum acceleration

}

namespace sensorOffsets
{
    constexpr double leftOffset = 2.0;
    constexpr double rightOffset = 2.0;
    constexpr double frontOffset = 3.0;
    constexpr double verticalOffset = 0.0;
    constexpr double horizontalOffset = 0.0;
}