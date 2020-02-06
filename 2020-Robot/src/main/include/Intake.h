/*
class Intake

    Public Methods

        void setSpeed(const double& = 0): Sets the speed of the intake
            motor. Defaults to 0.
*/

#pragma once

#include "rev/CANSparkMax.h"

class Intake {

    public:
        Intake(const int &intakeMotorCANID) {

            m_intakeMotor = new rev::CANSparkMax(intakeMotorCANID, rev::CANSparkMax::MotorType::kBrushless);
        }

        void setSpeed(const double &speedToSet = 0) {

            m_intakeMotor->Set(speedToSet);
        }

    private:
        rev::CANSparkMax *m_intakeMotor;
}