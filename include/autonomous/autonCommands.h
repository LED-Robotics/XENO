#pragma once

#include "autons.h"
#include "auton.h"
#include "command/includes.h"
#include "Left.h"

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
        case left_Auton:
            return Left::left();
        default:
            return new InstantCommand([]()
                                      { std::cout << "No auton" << std::endl; }, {});
        }
    }
};