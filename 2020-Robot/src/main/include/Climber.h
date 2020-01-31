#pragma once;

#include "rev/CANSparkMax.h"

class Climber {

    public:
        Climber(const int &climbMotorCANID) {

            climbMotor = new rev::CANSparkMax(climbMotorCANID, rev::CANSparkMax::MotorType::kBrushless);
        }

        void setSpeed(const double &speedToSet) {

            climbMotor->Set(speedToSet);
        }

    private:
        rev::CANSparkMax *climbMotor;
};