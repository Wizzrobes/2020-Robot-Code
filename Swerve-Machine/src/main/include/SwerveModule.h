/*
class SwerveModule

Constructors

    SwerveModule(const int&, const int&): Creates a swerve module with
        Spark MAX motor controllers on the two supplied CAN IDs, the first
        controlling drive, the second controlling swerve.

Public Methods

    void setDriveSpeed(const double&): Sets the driving speed to a double.
    void setSwerveSpeed(const double&): Sets the swerve speed to a double.
    double getDrivePosition(): Returns the total REV revolutions of the drive encoder.
    double getSwervePosition(): Returns the total REV revolutions of the swerve encoder.
    double getDriveSpeed(): Returns the speed of the drive encoder in RPM.
    double getSwerveSpeed(): Returns the speed of the swerve encoder in RPM.
    void assumeSwervePosition(const double& positionToAssume): Uses a mathematical function
        to assign a speed to the swerve motor to move quickly and accurately, within
        a tolerance, to any REV rotation value, clockwise or opposite.

    Note that the values returned by the get functions persist across disables, but
        not across power cycles.

Private Methods

    double calculateAssumePositionSpeed(const double&): Uses the following function

             {(1)/(1+e^((-1 * abs(z)) + 5)); z >= R_swerveTrainAssumePositionSpeedCalculationFirstEndBehaviorAt
        s(z)={R_swerveTrainAssumePositionSpeedCalculationFirstEndBehaviorSpeed; z < R_swerveTrainAssumePositionSpeedCalculationFirstEndBehaviorAt
             {R_swerveTrainAssumePositionSpeedCalculationSecondEndBehaviorSpeed; z < R_swerveTrainAssumePositionSpeedCalculationSecondEndBehaviorAt
            where
                s = speed at which the motor rotates to assume a position
                z = remaining REV revolutions of the position assumption
        to assign a speed with which to proceed towards the final position. It was developed,
        regressed, and tuned to move to the final position as fast as possible initially, slowing
        down as it approaches and becoming linear as it settles into tolerance at a high accuracy.

*/

#pragma once

#include <math.h>

#include "rev/CANSparkMax.h"

#include "RobotMap.h"

class SwerveModule {

    public:
        SwerveModule(const int &canDriveID, const int &canSwerveID) {

            m_driveMotor = new rev::CANSparkMax(canDriveID, rev::CANSparkMax::MotorType::kBrushless);
            m_driveMotorEncoder = new rev::CANEncoder(m_driveMotor->GetEncoder());
            m_swerveMotor = new rev::CANSparkMax(canSwerveID, rev::CANSparkMax::MotorType::kBrushless);
            m_swerveMotorEncoder = new rev::CANEncoder(m_swerveMotor->GetEncoder());

            //Allow the drive motor to coast, but brake the swerve motor for accuracy.
            m_driveMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
            m_swerveMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
        }

        void setDriveSpeed(const double &speedToSet) {

            m_driveMotor->Set(speedToSet);
        }
        void setSwerveSpeed(const double &speedToSet) {

            m_swerveMotor->Set(speedToSet);
        }
        double getDrivePosition() {

            return m_driveMotorEncoder->GetPosition();
        }
        double getSwervePosition() {

            return m_swerveMotorEncoder->GetPosition();
        }
        double getDriveSpeed() {

            return m_driveMotorEncoder->GetVelocity();
        }
        double getSwerveSpeed() {

            return m_swerveMotorEncoder->GetVelocity();
        }

        void assumeSwervePosition(const double& positionToAssume) {

            const double currentPosition = m_swerveMotorEncoder->GetPosition();
            //If the current position both CW and CCW is close enough to where we want to go (within one tolerance value)...
            if (positionToAssume - currentPosition < R_swerveTrainAssumePositionTolerance && currentPosition - positionToAssume < R_swerveTrainAssumePositionTolerance) {

                //Stop rotating the swerve motor and skip checking anything else...
                m_swerveMotor->Set(0);
            }
            //If the difference between where we want to be and where we are doesn't satisfy tolerance
            //(is more than one tolerance value away from 0, perfection)...
            else {

                //Rotate the swerve to the speed calculated by the mathematical function based on how
                //many REV revolutions are remaining towards 0, perfection...
                m_swerveMotor->Set(calculateAssumePositionSpeed(positionToAssume - currentPosition));
            }
        }

    private:
        rev::CANSparkMax *m_driveMotor;
        rev::CANEncoder *m_driveMotorEncoder;
        rev::CANSparkMax *m_swerveMotor;
        rev::CANEncoder *m_swerveMotorEncoder;

        double calculateAssumePositionSpeed(const double& howFarRemainingInTravel) {

            //Begin initally with a double calculated with the simplex function...
            double toReturn = ((1) / (1 + exp((-1 * abs(howFarRemainingInTravel)) + 5)));
            //If we satisfy conditions for the first linear piecewise, take that speed instead...
            if (abs(howFarRemainingInTravel) < R_swerveTrainAssumePositionSpeedCalculationFirstEndBehaviorAt) {

                toReturn = R_swerveTrainAssumePositionSpeedCalculationFirstEndBehaviorSpeed;
            }
            //Do the same for the second...
            if (abs(howFarRemainingInTravel) < R_swerveTrainAssumePositionSpeedCalculationSecondEndBehaviorAt) {

                toReturn = R_swerveTrainAssumePositionSpeedCalculationSecondEndBehaviorSpeed;
            }
            //And if we needed to travel negatively to get where we need to be, make the final speed negative...
            if (howFarRemainingInTravel < 0) {

                toReturn = -toReturn;
            }
            return toReturn;
        }
};
