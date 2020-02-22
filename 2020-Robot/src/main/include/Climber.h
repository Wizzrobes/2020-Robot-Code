/*
class Climber

    Constructors

        Climber(const int&)
            Create a climber with its motor as a Victor SPX speed controller
                on the supplied PWM Port.

    Public Methods

        void setSpeed(const int&, const double& = 0)
            Sets the speed of the supplied motor to the passed double
            (defaults to 0).

        enum LiftMotor
            Used to select which motor the set function operates on.
*/

#pragma once

#include <frc/VictorSP.h>

class Climber {

    public:
        Climber(const int &climbMotorPWMPort, const int &translateMotorPWMPort) {

            m_climbMotor = new frc::VictorSP(climbMotorPWMPort);
            m_translateMotor = new frc::VictorSP(translateMotorPWMPort);
        }

        void setSpeed(const int &motor, const double &speedToSet = 0) {

            switch (motor) {

                case Motor::kClimb: m_climbMotor->Set(speedToSet); break;
                case Motor::kTranslate: m_translateMotor->Set(speedToSet); break;
                case Motor::kBoth:
                    m_climbMotor->Set(speedToSet);
                    m_translateMotor->Set(speedToSet);
                    break;
            }
        }

        enum Motor {

            kClimb, kTranslate, kBoth
        };

    private:
        frc::VictorSP *m_climbMotor;
        frc::VictorSP *m_translateMotor;
};
