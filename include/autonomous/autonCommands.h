#pragma once

#include "autons.h"
#include "auton.h"
#include "command/includes.h"
#include "Left.h"

extern int AUTON;

/**
 * Allows easy selection of autonomous routines given a AUTON object, called on initialization to build states
 */
class AutonomousCommands
{
public:
    static Command *getAuton()
    {
        switch (AUTON)
        {
        case left:
            return Left::left();
        default:
            return new InstantCommand([]()
                                      { std::cout << "No auton" << std::endl; }, {});
        }
    }
};