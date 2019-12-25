/*
class SwerveModule

Constructors

    SwerveModule(const int&, const int&): Creates a swerve module with
        Spark MAX motor controllers on the two supplied CAN IDs, the first
        controlling drive, the second controlling swerve

*/

#pragma once

#include "rev/CANSparkMax.h"

class SwerveModule {

    public:
        SwerveModule(const int &canDriveID, const int &canSwerveID) {

            m_driveMotor = new rev::CANSparkMax(canDriveID, rev::CANSparkMax::MotorType::kBrushless);
            *m_driveMotorEncoder = m_driveMotor->GetEncoder();
            m_swerveMotor = new rev::CANSparkMax(canSwerveID, rev::CANSparkMax::MotorType::kBrushless);
            *m_swerveMotorEncoder = m_swerveMotor->GetEncoder();
        }


    private:
        rev::CANSparkMax *m_driveMotor;
        rev::CANEncoder *m_driveMotorEncoder;
        rev::CANSparkMax *m_swerveMotor;
        rev::CANEncoder *m_swerveMotorEncoder;
};