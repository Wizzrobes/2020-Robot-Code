/*
class Launcher

Constructors

    Launcher(const int&, const int&, const int&)
        Creates a launcher on CANIDs for its brushed index motor and its
        brushless launching motors, first then second.

Public Methods

    setIndexSpeed(const double& = 0)
        Sets the speed of the indexing motor. Defaults to zero.
    setLaunchSpeed(const double& = 0)
        Sets the speed of the launching motors. If the speed supplied
        is less than the global idling speed, sets that instead. Defaults
        to zero, which becomes a default to the idling speed.
*/

#pragma once

#include <rev/CANSparkMax.h>

class Launcher {

    public:
        Launcher(const int &indexMotorCANID, const int &launchMotorOneCANID, const int &launchMotorTwoCANID) {

            indexMotor = new rev::CANSparkMax(indexMotorCANID, rev::CANSparkMax::MotorType::kBrushed);
            launchMotorOne = new rev::CANSparkMax(launchMotorOneCANID, rev::CANSparkMax::MotorType::kBrushless);
            launchMotorTwo = new rev::CANSparkMax(launchMotorTwoCANID, rev::CANSparkMax::MotorType::kBrushless);
        }

        void setIndexSpeed(const double &speedToSet = 0) {

            //Both motors are mounted counterclockwise, so invert all numbers
            //to turn in the sensible direction.
            indexMotor->Set(-speedToSet);
        }
        void setLaunchSpeed(const double &speedToSet = 0) {

            //If the supplied speed is less than idling speed,
            //set idling speed instead.
            const double actualSpeedToSet = speedToSet < R_launcherDefaultSpeedLaunch ? R_launcherDefaultSpeedLaunchClose : speedToSet;
            launchMotorOne->Set(-speedToSet);
            //Invert the inversion for the second motor,
            //as they are mounted on opposite sides.
            launchMotorTwo->Set(speedToSet);
        }

    private:
        rev::CANSparkMax *indexMotor;
        rev::CANSparkMax *launchMotorOne;
        rev::CANSparkMax *launchMotorTwo;
};
