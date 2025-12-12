#include "autonomous_funcs.h"
#include <cmath>

double difference = 6.25; // inches
double difference2 = 6.5; // inches
double odomResetRight()
{
    double back = ((backRightDistance.get() + 31) / 25.4);
    double front = ((frontRightDistance.get()) / 25.4);

    double deltaDistance = (back - front) / (difference);
    double heading = 90 + (atan(deltaDistance) * (180 / M_PI));
    return heading;
}
double odomResetLeft()
{
    double back = ((backLeftDistance.get()) / 25.4);
    double front = ((frontLeftDistance.get() + 19) / 25.4);

    double deltaDistance = (back - front) / (difference2);
    double heading = 90 + (atan(deltaDistance) * (180 / M_PI));
    return heading;
}
double colorSorting(bool allianceId)
{
    bool alliance = allianceId;
    bool cubeColor = true;
    if (colorSort.get_hue() >= 240)
    {
        bool cubeColor = true; // true = blue idk how to give it blue and red instead of true and false
    }
    if (cubeColor != alliance)
    {
        l2Motor.move_relative(500, 127);
        l2Motor.move(-127);
    }
}