#pragma once

#include "subsystems/drivetrain.h"
#include "command/command.h"
#include "Eigen/Dense"

struct BezierAsset
{ /* placeholder for bezier data used by Ramsete */
};
inline const BezierAsset p_4_1_red{};
inline const BezierAsset p_4_1_blue{};
inline const BezierAsset p_4_2_red{};
inline const BezierAsset p_4_2_blue{};
inline const BezierAsset p_4_3_red{};
inline const BezierAsset p_4_3_blue{};
class Left
{
public:
    static Command *left()
    {
        Eigen::Vector3f startPose{(-10.0_in).getValue(), (55.0_in).getValue(), (53_deg).getValue()};

        drivetrainSubsystem->updateAllianceColor(startPose);
        const bool flip = ALLIANCE != RED;

        return new Sequence({
            // ...existing code...
            drivetrainSubsystem->setNorm(startPose.head<2>(), Eigen::Matrix2f::Identity() * 0.05, startPose.z(), flip),
            (new Rotate(drivetrainSubsystem, 195_deg, flip))->withTimeout(700_ms),
            drivetrainSubsystem->pct(0.07, 0.07)
            // ...existing code...
        });
    }
};