/*
class SwerveModule

Constructors

    SwerveTrain(SwerveModule*, SwerveModule*, SwerveModule*, SwerveModule*):
        Creates a swerve train with the swerve modules on the front right,
        front left, back left, and back right positions.

Public Methods

*/

#pragma once

#include "rev/CANSparkMax.h"

#include "SwerveModule.h"

class SwerveTrain {

    public:
        SwerveTrain(SwerveModule &frontRightModule, SwerveModule &frontLeftModule, SwerveModule &rearLeftModule, SwerveModule &rearRightModule) {

            frontRight = &frontRightModule;
            frontLeft = &frontLeftModule;
            rearLeft = &rearLeftModule;
            rearRight = &rearRightModule;
        }

    private:
        SwerveModule *frontRight;
        SwerveModule *frontLeft;
        SwerveModule *rearLeft;
        SwerveModule *rearRight;
};