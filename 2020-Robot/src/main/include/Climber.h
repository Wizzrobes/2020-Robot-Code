/*
class Climber

    Constructors

        Climber(const int&, const int&)
            Create a climber with its primary and secondary motors on the
                provided CAN Bus IDs.

    Public Methods

        void setSpeed(const int&, const double& = 0)
            Sets the speed of the supplied motor(s) to the passed double
            (defaults to 0).
        void getSpeed(const int&)
            Returns the encoder speed of the supplied motor in some REV value
            Note that if both motors are passed, zero is returned as an error!

        enum LiftMotor
            Used to select which motor the set and get functions operate on.
*/

#pragma once

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

            switch (motor) {

                case LiftMotor::kPrimary: m_primaryMotor->Set(speedToSet); break;
                case LiftMotor::kSecondary: m_secondaryMotor->Set(speedToSet); break;
                case LiftMotor::kBoth:
                    m_primaryMotor->Set(speedToSet);
                    m_secondaryMotor->Set(speedToSet);
                    break;
            }
        }

        double getSpeed(const int &motor) {
            
            if (motor == LiftMotor::kPrimary) {

                return m_primaryMotorEncoder->GetVelocity();
            }
            if (motor == LiftMotor::kSecondary) {

                return m_secondaryMotorEncoder->GetVelocity();
            }
            else {

                return 0;
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
