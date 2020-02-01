#pragma once;

#include "frc/smartdashboard/SmartDashboard.h"

#include "rev/CANSparkMax.h"

class Climber {

    public:
        Climber(const int &primaryMotorCANID, const int &secondaryMotorCANID) {

            m_primaryMotor = new rev::CANSparkMax(primaryMotorCANID, rev::CANSparkMax::MotorType::kBrushless);
            m_secondaryMotor = new rev::CANSparkMax(secondaryMotorCANID, rev::CANSparkMax::MotorType::kBrushless);

            m_primaryMotorEncoder = new rev::CANEncoder(m_primaryMotor->GetEncoder());
            m_secondaryMotorEncoder = new rev::CANEncoder(m_secondaryMotor->GetEncoder());

            m_primaryMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
            m_secondaryMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
        }

        void setSpeed(const int &motor, const double &speedToSet = 0) {

            if (motor == LiftMotor::kPrimary) {

                m_primaryMotor->Set(speedToSet);
            }
            if (motor == LiftMotor::kSecondary) {

                m_primaryMotor->Set(speedToSet);
            }
            if (motor == LiftMotor::kBoth) {

                m_primaryMotor->Set(speedToSet);
                m_secondaryMotor->Set(speedToSet);
            }
        }

        double getSpeed(const int &motor) {
            
            if (motor == LiftMotor::kPrimary) {

                return m_primaryMotorEncoder->GetVelocity();
            }
            if (motor == LiftMotor::kSecondary) {

                return m_secondaryMotorEncoder->GetVelocity();
            }
        }

        enum LiftMotor {

            kPrimary, kSecondary, kBoth
        };

    private:
        rev::CANSparkMax *m_primaryMotor;
        rev::CANSparkMax *m_secondaryMotor;

        rev::CANEncoder *m_primaryMotorEncoder;
        rev::CANEncoder *m_secondaryMotorEncoder;
};