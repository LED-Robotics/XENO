namespace DrivePorts
{
    constexpr int kFrontLeft = 11;
    constexpr int kFrontRight = -12;
    constexpr int kBackLeft = -13;
    constexpr int kBackRight = 18;
    constexpr int kMiddleRight = 19;
    constexpr int kMiddleLeft = -16;

}

namespace OdometryInfo
{
    constexpr double kTrackWidth = 14.0;
    constexpr float kWheelDiameter = lemlib::Omniwheel::OLD_325;
    constexpr double kWheelRPM = 450.0;
    constexpr double kMainGearRatio = 5 / 3;
    constexpr double kDriftConstant = 2.0;

}

namespace LateralPID
{                                                           // need to change
    constexpr double lateralkP = 12.0;                      // proportional gain
    constexpr double lateralkI = 0.0;                       // integral gain
    constexpr double lateralkD = 18.0;                      // derivative gain
    constexpr double lateralAntiWindup = 3.0;               // anti windup
    constexpr double lateralSmallErrorRange = 1.0;          // degrees
    constexpr double lateralSmallErrorRangeTimeout = 100.0; // milliseconds
    constexpr double lateralLargeErrorRange = 3.0;          // degrees
    constexpr double lateralLargeErrorRangeTimeout = 500.0; // milliseconds
    constexpr double lateralSlew = 20.0;                    // maximum acceleration

}

namespace AngularPID
{
    constexpr double angularkP = 3.0;                       // proportional gain
    constexpr double angularkI = 0.0;                       // integral gain
    constexpr double angularkD = 11.0;                      // derivative gain
    constexpr double angularAntiWindup = 3.0;               // anti windup
    constexpr double angularSmallErrorRange = 1.0;          // degrees
    constexpr double angularSmallErrorRangeTimeout = 100.0; // milliseconds
    constexpr double angularLargeErrorRange = 3.0;          // degrees
    constexpr double angularLargeErrorRangeTimeout = 500.0; // milliseconds
    constexpr double angularSlew = 0.0;                     // maximum acceleration

}