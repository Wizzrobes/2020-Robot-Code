/*
class SwerveModule

Constructors

    SwerveModule(const int&, const int&): Creates a swerve module with
        Spark MAX motor controllers on the two supplied CAN IDs, the first
        controlling drive, the second controlling swerve.

Public Methods

    void setDriveSpeed(const double&): Sets the driving speed to a double.
    void setSwerveSpeed(const double&): Sets the swerve speed to a double.
    double getDriveRevolutions(): Returns the total revolutions of the drive encoder.
    double getSwerveRevolutions(): Returns the total revolutions of the swerve encoder.
    double getDriveSpeed(): Returns the speed of the drive encoder in RPM.
    double getSwerveSpeed(): Returns the speed of the swerve encoder in RPM.

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

        void setDriveSpeed(const double &speedToSet) {

            m_driveMotor->Set(speedToSet);
        }
        void setSwerveSpeed(const double &speedToSet) {

            m_swerveMotor->Set(speedToSet);
        }
        double getDriveRevolutions() {

            return m_driveMotorEncoder->GetPosition();
        }
        double getSwerveRevolutions() {

            return m_swerveMotorEncoder->GetPosition();
        }
        double getDriveSpeed() {

            return m_driveMotorEncoder->GetVelocity();
        }
        double getSwerveSpeed() {

            return m_swerveMotorEncoder->GetVelocity();
        }


    private:
        rev::CANSparkMax *m_driveMotor;
        rev::CANEncoder *m_driveMotorEncoder;
        rev::CANSparkMax *m_swerveMotor;
        rev::CANEncoder *m_swerveMotorEncoder;
};