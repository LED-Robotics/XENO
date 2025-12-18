#pragma once

#include "autons.h"
#include "auton.h"
#include "command/includes.h"

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
        case SKILLS:
        default:
            return new InstantCommand([]()
                                      { std::cout << "No auton" << std::endl; }, {});
        }
    }
};
