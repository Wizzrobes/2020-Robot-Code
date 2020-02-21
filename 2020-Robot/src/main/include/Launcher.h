/*
class Launcher

Constructors

    Launcher(const int&, const int&)
        Creates a launcher on CANIDs for its brushed index motor and its
        brushless launching motor.

Public Methods

    setIndexSpeed(const double& = 0)
        Sets the speed of the indexing motor. Defaults to zero.
    setLaunchSpeed(const double& - 0)
        Sets the speed of the launching motor. Defaults to zero.
*/

#pragma once

#include <rev/CANSparkMax.h>

class Launcher {

    public:
        Launcher(const int &indexMotorCANID, const int &launchMotorCANID) {

            indexMotor = new rev::CANSparkMax(indexMotorCANID, rev::CANSparkMax::MotorType::kBrushed);
            launchMotor = new rev::CANSparkMax(launchMotorCANID, rev::CANSparkMax::MotorType::kBrushless);
        }

        void setIndexSpeed(const double &speedToSet = 0) {

            //Both motors are mounted counterclockwise, so invert all numbers
            //to turn in the sensible direction.
            indexMotor->Set(-speedToSet);
        }
        void setLaunchSpeed(const double &speedToSet = 0) {

            launchMotor->Set(-speedToSet);
        }

    private:
        rev::CANSparkMax *indexMotor;
        rev::CANSparkMax *launchMotor;
};
