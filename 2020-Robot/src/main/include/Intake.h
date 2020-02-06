#include "rev/CANSparkMax.h"
//this code will control the intake of the robot.

class Intake {

    public:
        Intake(const int &intakeMotorId) {

            m_intakeMotor = new rev::CANSparkMax(intakeMotorId, rev::CANSparkMax::MotorType::kBrushless);
        }

        void setSpeed(const double &speedToSet) {

            m_intakeMotor->Set(speedToSet);
        }

    private:
        rev::CANSparkMax *m_intakeMotor;

}