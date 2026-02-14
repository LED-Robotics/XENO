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

void standby()
{
}

void rightHalfRed()
{
    bunny_Ear_Piston.set_value(true);
    chassis.setPose(-47.25, -13.5, 180);                     // start pose
    chassis.moveToPoint(-47.25, -23, 500, {.maxSpeed = 80}); // Move off
    chassis.turnToHeading(90, 500, {}, false);               // Turn to blocks
    preRollers.move(127);
    l2Motor.move(-127);
    chassis.moveToPoint(-20, -23, 1000, {.maxSpeed = 60}, false); // Drive to balcaks
    chassis.moveToPoint(-42.0, -23, 1000, {.forwards = false, .maxSpeed = 80}, false);
    preRollers.move(0);
    l2Motor.move(0);
    chassis.turnToHeading(180, 500);
    chassis.moveToPoint(-42, -44, 1200, {.forwards = true, .maxSpeed = 80}, false);
    lw_Piston.set_value(true);       // move to wall
    chassis.turnToHeading(270, 500); // turn to move
    chassis.moveToPoint(-63, -46, 1000, {.forwards = true, .maxSpeed = 80}, false);
    // Go to matchloader
    preRollers.move(-127);
    delay(100);
    preRollers.move(127);
    l2Motor.move(127);
    preRollers.move(127);
    l2Motor.move(-127);
    delay(1000);
    preRollers.move(0);
    l2Motor.move(0);
    chassis.moveToPoint(-22, -44, 1250, {.forwards = false, .maxSpeed = 100}, false);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    // move to goal
    preRollers.move(-127);
    delay(100);
    preRollers.move(127);
    l2Motor.move(127);
    delay(3000);
    chassis.setPose(-22, -45, 270); // outtake like the nigga you are
    // chassis.moveToPoint(-32, -46, 1000, {.forwards = false, .maxSpeed = 100}, false); // move to goal
    chassis.turnToHeading(0, 500);
    chassis.moveToPoint(-32, -40.0, 1000, {.forwards = true, .maxSpeed = 100}, false); // move to goal
    chassis.turnToHeading(270, 500);
    chassis.moveToPoint(-5, -40.0, 1000, {.forwards = false}); // move to goal
    delay(500);
    bunny_Ear_Piston.set_value(false);
}
void leftHalfBlue()
{
    bunny_Ear_Piston.set_value(true);
    chassis.setPose(47.25, -13.5, 180);                     // start pose
    chassis.moveToPoint(47.25, -23, 500, {.maxSpeed = 80}); // Move off
    chassis.turnToHeading(270, 500, {}, false);             // Turn to blocks
    preRollers.move(127);                                   // THIS IS LEFT SIDE
    l2Motor.move(-127);
    chassis.moveToPoint(20, -23, 1250, {.maxSpeed = 70}, false); // Drive to balcaks
    chassis.moveToPoint(47.25, -23, 1250, {.forwards = false, .maxSpeed = 80}, false);
    preRollers.move(0);
    l2Motor.move(0);                                                                  // drive back
    chassis.turnToHeading(180, 500);                                                  // turn to move
    chassis.moveToPoint(47.25, -45, 1250, {.forwards = true, .maxSpeed = 80}, false); // move to wall
    chassis.turnToHeading(90, 500);
    lw_Piston.set_value(true); // turn to move
    chassis.moveToPoint(58, -45, 1250, {.forwards = true, .maxSpeed = 60}, false);
    preRollers.move(127); // THIS IS LEFT SIDE
    l2Motor.move(-127);   // Go to matchloader
    preRollers.move(127);
    l2Motor.move(-127);
    delay(1000);
    preRollers.move(0);
    l2Motor.move(0);
    chassis.moveToPoint(22, -45, 1250, {.forwards = false, .maxSpeed = 100}, false);
    // move to goal
    preRollers.move(-127);
    delay(100);
    preRollers.move(127);
    l2Motor.move(127);
    delay(2000);                                                                    // outtake like the nigga you are
    chassis.moveToPoint(30, -45, 1000, {.forwards = true, .maxSpeed = 100}, false); // move to goal
    // chassis.turnToHeading(0, 500);
    chassis.moveToPoint(22, -45, 1000, {.forwards = false, .minSpeed = 100}, false); // move to goal
    // chassis.turnToHeading(90, 500);
    // bunny_Ear_Piston.set_value(false);
    // chassis.moveToPoint(8, -34.5, 1250, {}, false); // move to goal
}
void soloAWPRed()
{
}
void soloAWPBlue()
{
}

void skills()
{
    bunny_Ear_Piston.set_value(true);
    chassis.setPose(-47.25, -15.32, 180);                             // start pose
    chassis.moveToPoint(-47.25, -48, 1000, {.maxSpeed = 100}, false); // Drive align with matchloader
    lw_Piston.set_value(true);
    delay(300);
    chassis.turnToHeading(270, 500, {}, false);                     // Turn to matchloader
    chassis.moveToPoint(-59.5, -48, 1500, {.maxSpeed = 60}, false); // Drive to matchloader
    double y = (100.75 + leftDistance.get_distance()) / 25.4;
    chassis.setPose(-59.5, -70 + y, 270);
    preRollers.move(127);
    l2Motor.move(-127);
    delay(200);
    chassis.moveToPoint(-57.5, -48, 200, {.maxSpeed = 100}, false);
    delay(1700);
    chassis.moveToPoint(-40, -48, 1000, {.forwards = false, .maxSpeed = 100}, false); // Drive from matchloader
    l2Motor.move(0);
    lw_Piston.set_value(false);
    chassis.turnToHeading(180, 500, {.maxSpeed = 100}, false);
    preRollers.move(0);                                                            // Turn to Goal
    chassis.moveToPoint(-40, -61, 1000, {}, false);                                // Drive to alley
    chassis.turnToHeading(90, 500, {.maxSpeed = 100}, false);                      // Turn to alley
    chassis.moveToPoint(40, -61, 2000, {.maxSpeed = 80}, false);                   // Drive through alley
    chassis.turnToHeading(0, 500, {.maxSpeed = 100}, false);                       // Turn to middle
    chassis.moveToPoint(40, -47, 1500, {.maxSpeed = 100}, false);                  // Drive align with matchloader
    chassis.turnToHeading(90, 500, {.maxSpeed = 100}, false);                      // Turn to matchloader
    chassis.moveToPoint(15, -47, 750, {.forwards = false, .maxSpeed = 60}, false); // Drive to goal
    preRollers.move(-127);
    l2Motor.move(-127);
    delay(50); // Anti Jam and Scoreee
    preRollers.move(127);
    l2Motor.move(127);
    delay(500);
    chassis.moveToPoint(16, -47, 200, {.forwards = false, .maxSpeed = 60}, false); // Drive to goal
    chassis.moveToPoint(15, -47, 200, {.forwards = false, .maxSpeed = 60}, false); // Drive to goal
    chassis.setPose(15, -47, 90);
    delay(2500);
    preRollers.move(0);
    l2Motor.move(0);
    lw_Piston.set_value(true);
    chassis.moveToPoint(57, -46, 1500, {.forwards = true, .maxSpeed = 60}, false); // Drive to goal
    preRollers.move(127);
    l2Motor.move(-127);
    delay(200);
    chassis.moveToPoint(54, -47, 200, {.forwards = true, .maxSpeed = 80}, false); // Drive to goal
    delay(2000);
    preRollers.move(0);
    l2Motor.move(0);

    chassis.moveToPoint(13, -47, 1500, {.forwards = false, .maxSpeed = 80}, false); // Drive to goal
    lw_Piston.set_value(false);
    y = (100.75 + rightDistance.get_distance()) / 25.4;
    chassis.setPose(13, -70 + y, 90);
    l2Motor.move(-127);
    delay(50); // Anti Jam and Score
    preRollers.move(127);
    l2Motor.move(127);
    delay(3500);
    preRollers.move(-127);
    l2Motor.move(127);
    chassis.moveToPoint(30, -47, 1000, {.maxSpeed = 100}, false); // Drive align with matchloader
    chassis.turnToHeading(0, 500);
    chassis.moveToPoint(30, 48, 2500, {.maxSpeed = 80}, false); // Drive align with matchloader
    lw_Piston.set_value(true);
    chassis.turnToHeading(90, 500, {.maxSpeed = 100}, false);   // Turn to matchloader
    chassis.moveToPoint(58, 48, 2000, {.maxSpeed = 60}, false); // Drive to matchloader
    preRollers.move(127);
    l2Motor.move(-127);
    delay(200);
    chassis.moveToPoint(57, 48, 300, {.maxSpeed = 100}, false);
    y = (100.75 + leftDistance.get_distance()) / 25.4;
    chassis.setPose(57, 70 - y, 90);
    delay(1700);
    preRollers.move(0);
    l2Motor.move(0);
    chassis.moveToPoint(30, 48, 1000, {.forwards = false, .maxSpeed = 100}, false); // Drive align with matchloader
    lw_Piston.set_value(false);
    chassis.turnToHeading(0, 500);
    chassis.moveToPoint(30, 61, 1000, {.maxSpeed = 100}, false); // Drive to wall
    chassis.turnToHeading(270, 500);
    chassis.moveToPoint(-30, 61, 2500, {.maxSpeed = 80}, false); // Drive through alley
    chassis.turnToHeading(0, 500);
    chassis.moveToPoint(-30, 48, 1000, {.forwards = false, .maxSpeed = 100}, false); // Drive align with goal
    chassis.turnToHeading(270, 500);
    preRollers.move(127);
    l2Motor.move(-127);
    chassis.moveToPoint(-10, 48, 1000, {.forwards = false, .maxSpeed = 100}, false); // Drive to goal
    chassis.setPose(-15, 48, 270);
    l2Motor.move(-127);
    delay(50); // Anti Jam and Score
    preRollers.move(127);
    l2Motor.move(127);
    delay(2500);
    preRollers.move(127);
    l2Motor.move(-127);
    lw_Piston.set_value(true);
    chassis.moveToPoint(-59, 49, 2000, {.maxSpeed = 60}, false); // Drive to match loader
    preRollers.move(127);
    l2Motor.move(-127);
    delay(200);
    // chassis.moveToPoint(-58, 49, 300, {.forwards = false, .maxSpeed = 100}, false);
    delay(1700);
    chassis.moveToPoint(-15, 49, 1000, {.forwards = false, .maxSpeed = 100}, false); // Drive to goal
    l2Motor.move(-127);
    delay(50); // Anti Jam and Score
    preRollers.move(127);
    l2Motor.move(127);
    delay(2500);
    preRollers.move(127);
    l2Motor.move(127);
    lw_Piston.set_value(false);
    // chassis.moveToPoint(-50, 30, 2000, {}, false); // Drive to goal
    // chassis.turnToHeading(180, 500);
    // preRollers.move(127);
    // l2Motor.move(127);
    // chassis.moveToPoint(-55, -10, 1000, {.maxSpeed = 80}, false); // Drive to goal
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