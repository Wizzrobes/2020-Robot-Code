/*
class Climber

Constructors

    Climber(const int&, const int&, const int&, const int&)
        Create a climber with its climb, translate, and wheel-rotate 
        motors as Victor SPX speed controller objects
        on the supplied PWM ports, and its servo for locking its ratchet
        on another PWM port. Take a final argument for a DIO pin to attach
        a limit switch to which denotes the bottom position of the arm.

Public Methods

    void setSpeed(const int&, const double& = 0)
        Sets the speed of the supplied LiftMotor to the passed double
        (defaults to 0).
    void lock(const bool& = true)
        If true, locks the climber in the up direction. If false or null,
        unlocks it. Persists across calls (it's hardware ;)). Defaults to
        locking.

    enum LiftMotor
        Used to select which motor the set function operates on.
*/

#pragma once

#include <frc/DigitalInput.h>
#include <frc/Servo.h>
#include <frc/VictorSP.h>

class Climber {

    public:
        Climber(const int &climbMotorPWMPort, const int &translateMotorPWMPort, const int &wheelMotorPWMPort, const int &servoPWMPort, const int &limitDIOPort) {

            m_climbMotor = new frc::VictorSP(climbMotorPWMPort);
            m_translateMotor = new frc::VictorSP(translateMotorPWMPort);
            m_wheelMotor = new frc::VictorSP(wheelMotorPWMPort);

            m_ratchetServo = new frc::Servo(servoPWMPort);

            m_limitBottom = new frc::DigitalInput(limitDIOPort);
        }

        void setSpeed(const int &motor, double speedToSet = 0) {

            switch (motor) {

                case Motor::kClimb:
                    //If a downward direction is wanted, do not allow it to
                    //happen if the bottom position is already reached.
                    if (speedToSet < 0) {

                        //Switches are normally open, so invert.
                        if (!m_limitBottom->Get()) {

                            speedToSet = 0;
                        }
                    }
                    m_climbMotor->Set(speedToSet);
                    break;
                //This motor is mounted upside-down, so invert it.
                case Motor::kTranslate: m_translateMotor->Set(-speedToSet); break;
                case Motor::kWheel: m_wheelMotor->Set(speedToSet); break;
                case Motor::kAll:
                    m_climbMotor->Set(speedToSet);
                    m_translateMotor->Set(speedToSet);
                    m_wheelMotor->Set(speedToSet);
                    break;
            }
        }

        void lock(const bool &action = true) {

            if (action) {

                m_ratchetServo->SetAngle(0);
            }
            else {

                m_ratchetServo->SetAngle(50);
            }
        }

        enum Motor {

            kClimb, kTranslate, kWheel, kAll
        };

    private:
        frc::VictorSP *m_climbMotor;
        frc::VictorSP *m_translateMotor;
        frc::VictorSP *m_wheelMotor;

        frc::Servo *m_ratchetServo;

        frc::DigitalInput *m_limitBottom;
};
