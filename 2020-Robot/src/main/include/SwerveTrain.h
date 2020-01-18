/*
class SwerveModule

Constructors

    SwerveTrain(SwerveModule*, SwerveModule*, SwerveModule*, SwerveModule*):
        Creates a swerve train with the swerve modules on the front right,
        front left, back left, and back right positions.

Public Methods

    void setDriveSpeed(const double&): Sets a speed to all driving motors on
        the train.
    void setSwerveSpeed(const double&): Sets a speed to all swerve motors on
        the train.
    void setSwerveZeroPosition(const bool&): Gets the current encoder values
        of the swerve motors and stores them as privates of the class. These
        are the values the swerve motors return to when invoking
        assumeSwerveZeroPosition().
        If the passed bool is true, publishes the stored data to the
        SmartDashboard. This is currently used for returning to and maintaining
        "straight".
    void assumeSwerveZeroPosition(): Drives the swerves to return to their
        zero position.
    void publishSwervePositions(): Puts the current swerve encoder positions
        to the SmartDashboard.
    void driveController(): Fully drives the swerve train on the supplied
        controller.

Private Methods

    double getControllerREVRotationsFromCenter(const double&, const double&):
        Discernes how many clockwise REV rotations from center the current
        location of the joystick is using vector trigonometry and properties.
        See https://en.wikipedia.org/wiki/Dot_product#Geometric_definition
    double getAbsoluteControllerMagnitude(frc::XboxController&): Gets the
        unsigned velocity of the control stick using only absolute value.
*/

#pragma once

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/XboxController.h>
#include "math.h"

#include "rev/CANSparkMax.h"

#include "SwerveModule.h"
#include "VectorDouble.h"

class SwerveTrain {

    public:
        SwerveTrain(SwerveModule &frontRightModule, SwerveModule &frontLeftModule, SwerveModule &rearLeftModule, SwerveModule &rearRightModule) {

            m_frontRight = &frontRightModule;
            m_frontLeft = &frontLeftModule;
            m_rearLeft = &rearLeftModule;
            m_rearRight = &rearRightModule;

            m_frontRightSwerveZeroPosition = 0;
            m_frontLeftSwerveZeroPosition = 0;
            m_rearLeftSwerveZeroPosition = 0;
            m_rearRightSwerveZeroPosition = 0;
        }

        void setDriveSpeed(const double& driveSpeed) {

            m_frontRight->setDriveSpeed(driveSpeed);
            m_frontLeft->setDriveSpeed(driveSpeed);
            m_rearLeft->setDriveSpeed(driveSpeed);
            m_rearRight->setDriveSpeed(driveSpeed);
        }

        void setSwerveSpeed(const double& swerveSpeed) {

            m_frontRight->setSwerveSpeed(swerveSpeed);
            m_frontLeft->setSwerveSpeed(swerveSpeed);
            m_rearLeft->setSwerveSpeed(swerveSpeed);
            m_rearRight->setSwerveSpeed(swerveSpeed);
        }

        void setSwerveZeroPosition(const bool &verbose = false) {

            m_frontRightSwerveZeroPosition = m_frontRight->getSwervePosition();
            m_frontLeftSwerveZeroPosition = m_frontLeft->getSwervePosition();
            m_rearLeftSwerveZeroPosition = m_rearLeft->getSwervePosition();
            m_rearRightSwerveZeroPosition = m_rearRight->getSwervePosition();

            if (verbose) {

                frc::SmartDashboard::PutNumber("FR Swrv Pos0", m_frontRightSwerveZeroPosition);
                frc::SmartDashboard::PutNumber("FL Swrv Pos0", m_frontLeftSwerveZeroPosition);
                frc::SmartDashboard::PutNumber("RL Swrv Pos0", m_rearLeftSwerveZeroPosition);
                frc::SmartDashboard::PutNumber("RR Swrv Pos0", m_rearRightSwerveZeroPosition);
            }
        }

        void setSwerveZeroPostion(const double &angle) {
            m_frontRightSwerveZeroPosition = angle; 
        }

        double getRelativeSwervePosition() {
            double position = m_frontRight->getSwervePosition() - m_frontRightSwerveZeroPosition;
            if(position > R_nicsConstant) {
                position = fmod(position, R_nicsConstant);
            } else {
                return position;
            }
        }

        double getNearestZeroPosition() {
            if(R_nicsConstant - getRelativeSwervePosition() < (R_nicsConstant/2.0)) {
                return R_nicsConstant;
            } else {
                return 0.0; 
            }
        }

        void assumeNearestSwerveZeroPosition() {
            double nearestPosition = getNearestZeroPosition(); 
            m_frontRight->assumeSwervePosition(nearestPosition);
            m_frontLeft->assumeSwervePosition(nearestPosition);
            m_rearLeft->assumeSwervePosition(nearestPosition);
            m_rearRight->assumeSwervePosition(nearestPosition);
        }

        void invertDriveMotors() {
            m_frontRight->invertDriveMotor(); 
            m_frontLeft->invertDriveMotor(); 
            m_rearLeft->invertDriveMotor(); 
            m_rearRight->invertDriveMotor();  
        }

        void assumeSwerveZeroPosition() {

            m_frontRight->assumeSwervePosition(m_frontRightSwerveZeroPosition);
            m_frontLeft->assumeSwervePosition(m_frontLeftSwerveZeroPosition);
            m_rearLeft->assumeSwervePosition(m_rearLeftSwerveZeroPosition);
            m_rearRight->assumeSwervePosition(m_rearRightSwerveZeroPosition);
        }
        void publishSwervePositions() {

            frc::SmartDashboard::PutNumber("FR Swrv Pos", m_frontRight->getSwervePosition());
            frc::SmartDashboard::PutNumber("FL Swrv Pos", m_frontLeft->getSwervePosition());
            frc::SmartDashboard::PutNumber("RL Swrv Pos", m_rearLeft->getSwervePosition());
            frc::SmartDashboard::PutNumber("RR Swrv Pos", m_rearRight->getSwervePosition());
        }
        void driveController(frc::XboxController *controller);

    private:
        SwerveModule *m_frontRight;
        SwerveModule *m_frontLeft;
        SwerveModule *m_rearLeft;
        SwerveModule *m_rearRight;

        double m_frontRightSwerveZeroPosition;
        double m_frontLeftSwerveZeroPosition;
        double m_rearLeftSwerveZeroPosition;
        double m_rearRightSwerveZeroPosition;

        double getControllerREVRotationsFromCenter(const double &x, const double &yInverted) {

            //Y seems to be inverted by default, so un-invert it...
            const double y = -yInverted;

            //Create vectors for the line x = 0 and the line formed by the joystick coordinates...
            VectorDouble center(0, 1);
            VectorDouble current(x, y);
            //Get the dot produt of the vectors for use in calculation...
            const double dotProduct = center * current;
            //Multiply each vector's magnitude together for use in calculation...
            const double magnitudeProduct = center.magnitude() * current.magnitude();
            //The cosine of the angle we want in rad is the dot product over the magnitude product...
            const double cosineAngle = dotProduct / magnitudeProduct;
            //The angle we want is the arccosine of its cosine...
            double angleRad = acos(cosineAngle);
            //To go from a full 0pi to 2pi and overcome the limitation of arccos, jump to 2pi and subtract the gradually decreasing angle...
            if (x < 0) {

                angleRad = (2 * M_PI) - angleRad;
            }
            //The decimal total of the whole circle is the radians over 2pi...
            double decimalTotalCircle = ((angleRad) / (2 * M_PI));
            //And the amount of REV rotations we want to rotate is the decimal total by Nic's Constant.
            double returnVal = decimalTotalCircle * R_nicsConstant;
            //If x is positive, invert it to rotate clockwise

                return returnVal;
        }
        double getAbsoluteControllerMagnitude(const double &x, const double &y) {

            //Get the absolute values of the joystick coordinates
            double absX = abs(x);
            double absY = abs(y);

            //Return the sum of the coordinates as a knock-off magnitude
            return absX + absY;
        }

        double getRadianAngleFromCenter(const double &x, const double &yInverted) {

            //Y seems to be inverted by default, so un-invert it...
            const double y = -yInverted;

            //Create vectors for the line x = 0 and the line formed by the joystick coordinates...
            VectorDouble center(0, 1);
            VectorDouble current(x, y);
            //Get the dot produt of the vectors for use in calculation...
            const double dotProduct = center * current;
            //Multiply each vector's magnitude together for use in calculation...
            const double magnitudeProduct = center.magnitude() * current.magnitude();
            //The cosine of the angle we want in rad is the dot product over the magnitude product...
            const double cosineAngle = dotProduct / magnitudeProduct;
            //The angle we want is the arccosine of its cosine...
            double angleRad = acos(cosineAngle);

            return angleRad;
        }
};
