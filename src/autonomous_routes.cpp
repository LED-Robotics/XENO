#include "autonomous_routes.h"

int autonomousSelected = 1;

void printData()
{
    pros::lcd::clear_line(4);
    switch (autonomousSelected)
    {
    case 0:
        pros::lcd::print(4, "Standby");
        break;
    case 1:
        pros::lcd::print(4, "1 RING BABY");
        break;
    case 2:
        pros::lcd::print(4, "Blue WP");
        break;
    case 3:
        pros::lcd::print(4, "Red Auto");
        break;
    case 4:
        pros::lcd::print(4, "Red WP");
        break;
    case 5:
        pros::lcd::print(4, "Skills");
        break;
    }
}

void lcdAutonSelect()
{
    autonomousSelected++;
    if (autonomousSelected > 1)
    {
        autonomousSelected = 0;
    }
    printData();
}

void lcdAllianceSelect()
{
    // if (alliance == RED_ID) {
    //     alliance = BLUE_ID;
    // } else if (alliance == BLUE_ID) {
    //     alliance = RED_ID;
    // }
    // printData();
}

struct Point
{
    float x = 0.0;
    float y = 0.0;
};

Point makePoint(float x = 0.0, float y = 0.0)
{
    return Point{x + 72.0f, y + 72.0f};
}

void rightHalf()
{
    chassis.setPose(-44, -4, 120);
    preRollers.move(127);
    l2Motor.move(-90);
    chassis.moveToPoint(-10, -36, 2500, {.forwards = true, .maxSpeed = 50}, false);
    lw_Piston.set_value(true);
}
void soloAWP()
{
    chassis.setPose(-45, 10, 270);
    chassis.moveToPoint(-14, 10, 1500, {.forwards = false, .maxSpeed = 80}, false);
    l2Motor.move(90);
    delay(500);
    l2Motor.move(-127);
    chassis.moveToPoint(-23, 10, 600, {.forwards = true, .maxSpeed = 80}, false);
    chassis.turnToHeading(180, 800, {}, false);
    double heading = (odomResetRight() + 90);
    preRollers.move(127);
    chassis.setPose(-21, 10, heading);
    chassis.moveToPoint(-23, -17, 2500, {.forwards = true, .maxSpeed = 50}, false);
    preRollers.move(-100);
    chassis.turnToHeading(45, 700, {}, false);
    preRollers.move(-100);
    chassis.moveToPoint(-14, -8.5, 500, {.forwards = true, .maxSpeed = 80}, false);
    delay(1250);
    preRollers.move(127);
    chassis.moveToPoint(-45, -48, 2000, {.forwards = false, .maxSpeed = 80}, false);
    double heading2 = (odomResetRight());
    chassis.setPose(-45, -47.5, heading2);
    chassis.turnToHeading(270, 1000, {}, false);
    chassis.moveToPoint(-40, -47.5, 250, {.forwards = false, .maxSpeed = 127});
    lw_Piston.set_value(true);
    chassis.moveToPoint(-58, -47.5, 550, {.forwards = true, .maxSpeed = 80}, false);
    double leftDis = ((frontLeftDistance.get_distance() + 19) / 25.4);
    double heading3 = (odomResetLeft());
    delay(1000);
    chassis.setPose(-58, leftDis - 70, 270);
    chassis.moveToPoint(-20, -47.5, 750, {.forwards = false, .maxSpeed = 100}, false);
    l2Motor.move(127);
}
void soloAWPBlue()
{
    chassis.setPose(45, 10, 90);
    chassis.moveToPoint(14, 10, 1500, {.forwards = false, .maxSpeed = 80}, false);
    l2Motor.move(90);
    delay(500);
    l2Motor.move(-127);
    chassis.moveToPoint(23, 10, 600, {.forwards = true, .maxSpeed = 80}, false);
    chassis.turnToHeading(0, 800, {}, false);
    double heading = (odomResetRight() - 90);
    preRollers.move(127);
    chassis.setPose(21, 10, heading);
    chassis.moveToPoint(23, -17, 2500, {.forwards = true, .maxSpeed = 50}, false);
    preRollers.move(-100);
    chassis.turnToHeading(225, 700, {}, false);
    preRollers.move(-100);
    chassis.moveToPoint(14, -8.5, 500, {.forwards = true, .maxSpeed = 80}, false);
    delay(1250);
    preRollers.move(127);
    chassis.moveToPoint(45, -48, 2000, {.forwards = false, .maxSpeed = 80}, false);
    double heading2 = (odomResetRight());
    chassis.setPose(45, -47.5, heading2);
    chassis.turnToHeading(270, 1000, {}, false);
    chassis.moveToPoint(40, -47.5, 250, {.forwards = false, .maxSpeed = 127});
    lw_Piston.set_value(true);
    chassis.moveToPoint(58, -47.5, 550, {.forwards = true, .maxSpeed = 80}, false);
    double leftDis = ((frontLeftDistance.get_distance() + 19) / 25.4);
    double heading3 = (odomResetLeft());
    delay(1000);
    chassis.setPose(58, leftDis - 70, 270);
    chassis.moveToPoint(20, -47.5, 750, {.forwards = false, .maxSpeed = 100}, false);
    l2Motor.move(127);
}
void executeAutonomous()
{
    switch (autonomousSelected)
    {
    case 0:
        break;
    case 1:
        break;
    case 2:
        break;
    case 3:
        break;
    case 4:
        break;
    case 5:
        break;
    }
}