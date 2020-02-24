/*
class Climber

Constructors

    Climber(const int&, const int&, const int&)
        Create a climber with its climb, translate, and wheel-rotate 
        motors as Victor SPX speed controller objects
        on the supplied PWM ports.

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
        Climber(const int &climbMotorPWMPort, const int &translateMotorPWMPort, const int &wheelMotorPWMPort) {

            m_climbMotor = new frc::VictorSP(climbMotorPWMPort);
            m_translateMotor = new frc::VictorSP(translateMotorPWMPort);
            m_wheelMotor = new frc::VictorSP(wheelMotorPWMPort);
        }

        void setSpeed(const int &motor, const double &speedToSet = 0) {

            switch (motor) {

                case Motor::kClimb: m_climbMotor->Set(speedToSet); break;
                case Motor::kTranslate: m_translateMotor->Set(speedToSet); break;
                case Motor::kWheel: m_wheelMotor->Set(speedToSet); break;
                case Motor::kBoth:
                    m_climbMotor->Set(speedToSet);
                    m_translateMotor->Set(speedToSet);
                    m_wheelMotor->Set(speedToSet);
                    break;
            }
        }

        enum Motor {

            kClimb, kTranslate, kWheel, kBoth
        };

    private:
        frc::VictorSP *m_climbMotor;
        frc::VictorSP *m_translateMotor;
        frc::VictorSP *m_wheelMotor;
};
