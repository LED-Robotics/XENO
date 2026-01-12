#include "autonomous_funcs.h"
#include <cmath>

double frontOffset = 0.0;
double leftOffset = 0.0;
double rightOffset = 0.0;
double backOffset = 0.0;
double odomResetFront()
{
    double frontDis = frontDistance.get_distance() / 25.4; // convert to inches
    double adjustedFront = frontDis + frontOffset;
    return adjustedFront;
}
double odomResetLeft()
{
    double leftDis = leftDistance.get_distance() / 25.4; // convert to inches
    double adjustedLeft = leftDis + leftOffset;
    return adjustedLeft;
}
double odomResetRight()
{
    double rightDis = rightDistance.get_distance() / 25.4; // convert to inches
    double adjustedRight = rightDis + rightOffset;
    return adjustedRight;
}
double odomResetBack()
{
    double backDis = backDistance.get_distance() / 25.4; // convert to inches
    double adjustedBack = backDis + backOffset;
    return adjustedBack;
}
double colorSorting(bool allianceId)
{
}