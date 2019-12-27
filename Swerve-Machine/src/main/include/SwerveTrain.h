#pragma once

#include "rev/CANSparkMax.h"

#include "../include/SwerveModule.h"

class SwerveTrain {

    public:
        SwerveTrain(SwerveModule *frontRightModule, SwerveModule *frontLeftModule, SwerveModule *backLeftModule, SwerveModule *backRightModule) {

            frontRight = frontRightModule;
            frontLeft = frontLeftModule;
            backLeft = backLeftModule;
            backRight = backRightModule;
        }

    private:
        SwerveModule *frontRight;
        SwerveModule *frontLeft;
        SwerveModule *backLeft;
        SwerveModule *backRight;
};