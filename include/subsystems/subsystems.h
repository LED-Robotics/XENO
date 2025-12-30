#pragma once

#include "Intake.h"
#include "command/commandController.h"
#include "command/commandScheduler.h"
#include "command/conditionalCommand.h"
#include "command/parallelCommandGroup.h"
#include "command/parallelRaceGroup.h"
#include "command/proxyCommand.h"
#include "command/repeatCommand.h"
#include "command/scheduleCommand.h"
#include "command/sequence.h"
#include "command/waitCommand.h"
#include "command/waitUntilCommand.h"
#include "commands/driveMove.h"
#include "commands/ramsete.h"
#include "commands/rotate.h"
#include "drivetrain.h"
#include "localization/distance.h"
#include "motionProfiling/pathCommands.h"
#include "pros/adi.hpp"
#include "solenoidSubsystem.h"

#include <algorithm>
#include <queue>

inline DrivetrainSubsystem *drivetrainSubsystem;
inline MotorSubsystem *conveyerSubsystem;
inline MotorSubsystem *hoodSubsystem;
inline SolenoidSubsystem *descoreSubsystem;
inline SolenoidSubsystem *doubleParkSubsystem;
inline SolenoidSubsystem *storingSubsystem;
inline SolenoidSubsystem *l2Subsystem;
inline SolenoidSubsystem *matchLoaderSubsystem;

// ADD COMMANDS HERE

inline CommandController primary(pros::controller_id_e_t::E_CONTROLLER_MASTER);
inline CommandController partner(pros::controller_id_e_t::E_CONTROLLER_PARTNER);

bool hangReleased = false;

inline void initializeController()
{

    // Intake Controls

    // Outtake

    primary.getTrigger(DIGITAL_R1)
        ->andOther(primary.getTrigger(DIGITAL_R2)->negate())
        ->andOther(primary.getTrigger(DIGITAL_L1)->negate())
        ->andOther(primary.getTrigger(DIGITAL_L2)->negate())
        ->toggleOnTrue(conveyerSubsystem->pctCommand(-1.0))
        ->toggleOnTrue(hoodSubsystem->pctCommand(-1.0));

    // Score L2

    primary.getTrigger(DIGITAL_L1)
        ->andOther(primary.getTrigger(DIGITAL_L2)->negate())
        ->andOther(primary.getTrigger(DIGITAL_R1)->negate())
        ->andOther(primary.getTrigger(DIGITAL_R2)->negate())
        ->toggleOnTrue(conveyerSubsystem->pctCommand(0.8))
        ->toggleOnTrue(l2Subsystem->levelCommand(true));

    // Score L2: toggle L2 solenoid when L1 pressed

    primary.getTrigger(DIGITAL_L2)
        ->andOther(primary.getTrigger(DIGITAL_L1)->negate())
        ->andOther(primary.getTrigger(DIGITAL_R1)->negate())
        ->andOther(primary.getTrigger(DIGITAL_R2)->negate())
        ->toggleOnTrue(conveyerSubsystem->pctCommand(1.0))
        ->toggleOnTrue(hoodSubsystem->pctCommand(1.0));

    primary.getTrigger(DIGITAL_R2)
        ->andOther(primary.getTrigger(DIGITAL_R1)->negate())
        ->andOther(primary.getTrigger(DIGITAL_L1)->negate())
        ->andOther(primary.getTrigger(DIGITAL_L2)->negate())
        ->toggleOnTrue(conveyerSubsystem->pctCommand(1.0))
        ->toggleOnTrue(hoodSubsystem->pctCommand(-1.0));

    // Pid Tuning

    primary.getTrigger(DIGITAL_LEFT)->whileTrue(drivetrainSubsystem->characterizeAngular());
    primary.getTrigger(DIGITAL_UP)->whileTrue(drivetrainSubsystem->characterizeLinear());

    auto *compTrigger = new Trigger([]()
                                    { return pros::c::competition_is_field(); });

    // Match Loader Control

    primary.getTrigger(DIGITAL_DOWN)
        ->toggleOnTrue(matchLoaderSubsystem->levelCommand(true));

    // Descore Control

    primary.getTrigger(DIGITAL_A)
        ->onTrue(descoreSubsystem->levelCommand(false))
        ->onFalse(descoreSubsystem->levelCommand(true));

    // Double Park
};

inline void initializePathCommands()
{
}

inline void initializeCommands()
{
}

inline void subsystemInit()
{
    TELEMETRY.setSerial(new pros::Serial(0, 921600));

    // 'd' is hang - done
    // 'a' is hang - done
    // 'e' is back clamp - done
    // 'c' is pto - done
    // 'b' is potentiometer - done
    // 5 is inertial
    // 6 is top intake motor
    // 3 is rotation on odom
    // 2 is right distance sensor
    // 9 is back distance sensor
    // 19 is left distance sensor
    // 20 is the front distance sensor
    // 15 is lb oppisite of last bot
    // 12 is bottom intake
    // 4 is back right drive motor - reversed
    // 7 is middle drive mtoro right
    // 10 is front drive motor right
    // 16 is back left drive motor - reversed
    // 17 is front drive motor left
    // 18 middle drive motor left

    conveyerSubsystem = new MotorSubsystem(pros::Motor(-6));
    hoodSubsystem = new MotorSubsystem(pros::Motor(7));
    descoreSubsystem = new SolenoidSubsystem(pros::adi::DigitalOut('e'));
    doubleParkSubsystem = new SolenoidSubsystem(pros::adi::DigitalOut('a'));
    storingSubsystem = new SolenoidSubsystem(pros::adi::DigitalOut('c'));
    l2Subsystem = new SolenoidSubsystem(pros::adi::DigitalOut('b'));
    matchLoaderSubsystem = new SolenoidSubsystem(pros::adi::DigitalOut('d'));
    drivetrainSubsystem = new DrivetrainSubsystem(
        {-8, 9, 10},
        {1, -2, -3},
        pros::Imu(4),
        pros::Rotation(5)); // wheels listed back to front; 8 for rotation sensor on pto

    pros::Task([]()
               {
        if (!drivetrainSubsystem->odomConnected()) {
            primary.rumble("-- --");
            pros::delay(2000);
        }

        if (pros::battery::get_capacity() < 50.0) {
            primary.rumble("..-");
            pros::delay(2000);
        }

        // Check motor temps
        if (std::max({drivetrainSubsystem->getTopMotorTemp(),
                      conveyerSubsystem->getTopMotorTemp()}) >= 45.0) {
            primary.rumble(".--");
        } });

    drivetrainSubsystem->addLocalizationSensor(new Distance(CONFIG::DISTANCE_LEFT_OFFSET, 0.987, pros::Distance(19)));
    drivetrainSubsystem->addLocalizationSensor(new Distance(CONFIG::DISTANCE_FRONT_OFFSET, 0.986, pros::Distance(20)));
    drivetrainSubsystem->addLocalizationSensor(new Distance(CONFIG::DISTANCE_RIGHT_OFFSET, 0.980, pros::Distance(2)));
    drivetrainSubsystem->addLocalizationSensor(new Distance(CONFIG::DISTANCE_BACK_OFFSET, 0.979, pros::Distance(9)));

    drivetrainSubsystem->initUniform(-70_in, -70_in, 70_in, 70_in, 0_deg, false);

    CommandScheduler::registerSubsystem(drivetrainSubsystem, drivetrainSubsystem->arcade(primary));
    CommandScheduler::registerSubsystem(conveyerSubsystem, conveyerSubsystem->stopIntake());
    CommandScheduler::registerSubsystem(hoodSubsystem, hoodSubsystem->stopIntake());
    CommandScheduler::registerSubsystem(descoreSubsystem, descoreSubsystem->levelCommand(false));
    CommandScheduler::registerSubsystem(doubleParkSubsystem, doubleParkSubsystem->levelCommand(false));
    CommandScheduler::registerSubsystem(matchLoaderSubsystem, matchLoaderSubsystem->levelCommand(false));
    CommandScheduler::registerSubsystem(l2Subsystem, l2Subsystem->levelCommand(true));
    CommandScheduler::registerSubsystem(storingSubsystem, storingSubsystem->levelCommand(false));

    initializeCommands();
    initializeController();
    initializePathCommands();
}
