/*
class Climber

Constructors

    Climber(const int&, const int&, const int&)
        Create a climber with its climb, translate, and wheel-rotate 
        motors as Victor SPX speed controller objects
        on the supplied PWM ports, and its servo for locking its ratchet
        on another PWM port.

Public Methods

    void setSpeed(const int&, const double& = 0)
        Sets the speed of the supplied LiftMotor to the passed double
        (defaults to 0).

    void unlock(const bool& = false)
        If true, unlocks the motor to move the arm up. If false or null,
        locks it. Persists across calls (it's hardware ;))

    enum LiftMotor
        Used to select which motor the set function operates on.
*/

#pragma once

#include <frc/Servo.h>
#include <frc/VictorSP.h>

class Climber {

    public:
        Climber(const int &climbMotorPWMPort, const int &translateMotorPWMPort, const int &wheelMotorPWMPort, const int &servoPWMPort) {

            m_climbMotor = new frc::VictorSP(climbMotorPWMPort);
            m_translateMotor = new frc::VictorSP(translateMotorPWMPort);
            m_wheelMotor = new frc::VictorSP(wheelMotorPWMPort);

            m_ratchetServo = new frc::Servo(servoPWMPort);
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
        void unlock(const bool &action = false) {

            if (action) {

                m_ratchetServo->SetAngle(15);
            }
            else {

                m_ratchetServo->SetAngle(0);
            }
        }

        enum Motor {

            kClimb, kTranslate, kWheel, kBoth
        };

    private:
        frc::VictorSP *m_climbMotor;
        frc::VictorSP *m_translateMotor;
        frc::VictorSP *m_wheelMotor;

        frc::Servo *m_ratchetServo;
};
