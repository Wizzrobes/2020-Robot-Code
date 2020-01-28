#pragma once

#include <rev/CANSparkMax.h>

class Launcher {

    public:
        Launcher(const int &indexMotorCANID, const int &launchMotorCANID) {

            indexMotor = new rev::CANSparkMax(indexMotorCANID, rev::CANSparkMax::MotorType::kBrushless);
            launchMotor = new rev::CANSparkMax(launchMotorCANID, rev::CANSparkMax::MotorType::kBrushless);
        }

        void setIndexSpeed(const double &speedToSet) {

            indexMotor->Set(speedToSet);
        }
        void setLaunchSpeed(const double &speedToSet) {

            launchMotor->Set(speedToSet);
        }

    private:
        rev::CANSparkMax *indexMotor;
        rev::CANSparkMax *launchMotor;
};
