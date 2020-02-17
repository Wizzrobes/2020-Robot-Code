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
