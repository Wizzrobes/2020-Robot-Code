#pragma once;

#include "rev/CANSparkMax.h"
#include "frc/smartdashboard/SmartDashboard.h"

class Climber {

    public:
        Climber(const int &climbMotorCANID) {

            climbMotor = new rev::CANSparkMax(climbMotorCANID, rev::CANSparkMax::MotorType::kBrushless);
            climbMotorEncoder = new rev::CANEncoder(climbMotor->GetEncoder());
            climbMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
        }

        void setSpeed(const double &speedToSet) {

            climbMotor->Set(speedToSet);
        }

        double getSpeed() {
            
            return climbMotor->Get();
        }

        double getRPM() {

            return climbMotorEncoder->GetVelocity();
        }

    private:
        rev::CANSparkMax *climbMotor;
        rev::CANEncoder *climbMotorEncoder;
};