#include "teleop.h"
#include "lemlib/api.hpp"
#include "robot_map.h"

void teleopDrive()
{
    double x = (double)master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) / 127.0; // /127 to push value between -1.0 and 1.0
    double y = (double)master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) / 127.0;
    // zero out axes if they fall within deadzone
    if (x > -0.05 && x < 0.05)
        x = 0.0;
    if (y > -0.05 && y < 0.05)
        y = 0.0;

    // setup differential variables for arcade control
    double leftSpeedRaw = y + x;
    double rightSpeedRaw = y - x;

    // put speeds through a polynomial to smooth out joystick input
    // check the curve out here: https://www.desmos.com/calculator/65tpwhxyai the range between 0.0 to 1.0 is used for the motors
    // change driveCurveExtent to modify curve strength
    double leftSpeed = 0.95 * pow(leftSpeedRaw, 3) + (1 - 0.95) * leftSpeedRaw;
    double rightSpeed = 0.95 * pow(rightSpeedRaw, 3) + (1 - 0.95) * rightSpeedRaw;

    // set motors to final speeds
    leftMotors.move((int32_t)(leftSpeed * 127));
    rightMotors.move((int32_t)(rightSpeed * 127));

    // delay to save resources
    pros::delay(25);
}
bool intakeOn = false;

void intake()
{
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2))
    {
        if (intakeOn == false)
        {
            intakeOn = true;
            preRollers.move(127);
            l2Motor.move(127);
        }
        else if (intakeOn == true)
        {
            intakeOn = false;
            preRollers.move(0);
            l2Motor.move(0);
        }
    }
    else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1))
    {
        if (intakeOn == false)
        {
            intakeOn = true;
            preRollers.move(-127);
            l2Motor.move(-127);
        }
        else if (intakeOn == true)
        {
            intakeOn = false;
            preRollers.move(0);
            l2Motor.move(0);
        }
    }
    else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2))
    {
        if (intakeOn == false)
        {
            intakeOn = true;
            preRollers.move(127);
            l2Motor.move(-127);
        }
        else if (intakeOn == true)
        {
            intakeOn = false;
            preRollers.move(0);
            l2Motor.move(0);
        }
    }
}

void storageSystem()
{
    // if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
    // {
    //     l2Motor.move(127);
    // }
    // else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
    // {
    //     l2Motor.move(-127);
    // }
    // else
    // {
    //     l2Motor.move(0);
    // }
}

bool lw = false;
void littleWill()
{
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN))
    {
        if (lw == false)
        {
            lw = true;
            lw_Piston.set_value(true);
        }
        else if (lw == true)
        {
            lw = false;
            lw_Piston.set_value(false);
        }
    }
}
bool hoodOn = false;
void hood()
{
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B))
    {
        if (hoodOn == false)
        {
            hoodOn = true;
            hood_Piston.set_value(true);
        }
        else if (hoodOn == true)
        {
            hoodOn = false;
            hood_Piston.set_value(false);
        }
    }
}

void intakeScore()
{
}

bool dp = false;
void doublePark()
{
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT))
    {
        if (dp == false)
        {
            dp = true;
            dp_Piston.set_value(true);
        }
        else if (dp == true)
        {
            dp = false;
            dp_Piston.set_value(false);
        }
    }
}
