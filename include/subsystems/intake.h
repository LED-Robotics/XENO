#pragma once
#include "config.h"

#include "command/command.h"
#include "command/runCommand.h"

#include "pros/motors.hpp"

class MotorSubsystem : public Subsystem
{
    pros::Motor intakeMotorLeft;
    pros::Motor intakeMotorRight;

public:
    explicit MotorSubsystem(pros::Motor intake_motor_left, pros::Motor intake_motor_right) : intakeMotorLeft(std::move(intake_motor_left)),
                                                                                             intakeMotorRight(std::move(intake_motor_right))
    {
        intakeMotorLeft.set_encoder_units_all(pros::MotorEncoderUnits::rotations);
        intakeMotorRight.set_encoder_units_all(pros::MotorEncoderUnits::rotations);
    }

    void periodic() override
    {
        // No-op
    }

    void setPct(const double pct)
    {
        this->intakeMotorLeft.move_voltage(pct * 12000.0);
        this->intakeMotorRight.move_voltage(pct * 12000.0);
    }

    void setSpeed(double speed)
    {
        this->intakeMotorLeft.move_velocity(speed);
        this->intakeMotorRight.move_velocity(speed);
    }

    RunCommand *stopIntake()
    {
        return new RunCommand([this]()
                              { this->setPct(0.0); }, {this});
    }

    RunCommand *pctCommand(double pct)
    {
        return new RunCommand([this, pct]()
                              { this->setPct(pct); }, {this});
    }

    double getTopMotorTemp() const
    {
        return this->intakeMotorLeft.get_temperature();
        return this->intakeMotorRight.get_temperature();
    }

    ~MotorSubsystem() override = default;
};
