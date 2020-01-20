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
    void assumeSwerveNearestZeroPosition(): Drives the swerves to the
        nearest Nic's Constant multiple of its zero value, CW or CCW.
        In doing so, really only drives the swerve to either 0 or
        the value of one Nic's Constant. Since the pathfinding function
        thinks clockwise, this ends up driving it to zero the fastest
        way possible, making use of getSwervePositionSingleRotation() as
        the driver function to make this possible. assumeSwerveZeroPosition()
        cannot make this optimization, and simply goes to whatever the zero
        value is. Useful for low-level things.
    void publishSwervePositions(): Puts the current swerve encoder positions
        to the SmartDashboard.
    void driveController(): Fully drives the swerve train on the supplied
        controller.

Private Methods

    double getControllerREVRotationsFromCenter(frc::XboxController*):
        Discernes how many clockwise REV rotations from center the current
        location of the joystick is using vector trigonometry and properties.
        See https://en.wikipedia.org/wiki/Dot_product#Geometric_definition
    double getControllerAngleFromCenter(frc::XboxController*): Same
        as above, but simply returns the radian angle (no conversion
        back to a REV Rotation value).
    double getAbsoluteControllerMagnitude(frc::XboxController&): Gets the
        unsigned velocity of the control stick using only absolute value.
    bool getControllerAllInDeadzone(frc::XboxController*): Returns
        true if each stick is within the deadzone specified in RobotMap,
        false otherwise.
*/

#pragma once

#include <math.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Joystick.h>

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

            m_frontRight->setZeroPosition();
            m_frontLeft->setZeroPosition();
            m_rearLeft->setZeroPosition();
            m_rearRight->setZeroPosition();

            if (verbose) {

                frc::SmartDashboard::PutNumber("FR Swrv Pos0", m_frontRight->getSwerveZeroPosition());
                frc::SmartDashboard::PutNumber("FL Swrv Pos0", m_frontLeft->getSwerveZeroPosition());
                frc::SmartDashboard::PutNumber("RL Swrv Pos0", m_rearLeft->getSwerveZeroPosition());
                frc::SmartDashboard::PutNumber("RR Swrv Pos0", m_rearRight->getSwerveZeroPosition());
            }
        }
        void assumeZeroPosition() {

            m_frontRight->assumeSwerveZeroPosition();
            m_frontLeft->assumeSwerveZeroPosition();
            m_rearLeft->assumeSwerveZeroPosition();
            m_rearRight->assumeSwerveZeroPosition();
        }
        void assumeNearestZeroPosition() {

            m_frontRight->assumeSwerveNearestZeroPosition();
            m_frontLeft->assumeSwerveNearestZeroPosition();
            m_rearLeft->assumeSwerveNearestZeroPosition();
            m_rearRight->assumeSwerveNearestZeroPosition();
        }

        void publishSwervePositions() {

            frc::SmartDashboard::PutNumber("FR Swrv Pos", m_frontRight->getSwervePosition());
            frc::SmartDashboard::PutNumber("FL Swrv Pos", m_frontLeft->getSwervePosition());
            frc::SmartDashboard::PutNumber("RL Swrv Pos", m_rearLeft->getSwervePosition());
            frc::SmartDashboard::PutNumber("RR Swrv Pos", m_rearRight->getSwervePosition());
        }

        void driveController(frc::Joystick *controller);

    private:
        SwerveModule *m_frontRight;
        SwerveModule *m_frontLeft;
        SwerveModule *m_rearLeft;
        SwerveModule *m_rearRight;

        double getControllerREVRotationsFromCenter(frc::Joystick *controller) {

            //TODO: Why is there a negative here?
            //took out negative to test with joystick
            const double x = -controller->GetX(frc::GenericHID::kRightHand);
            //Y seems to be inverted by default, so un-invert it...
            const double y = -controller->GetY(frc::GenericHID::kRightHand);

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
            return decimalTotalCircle * R_nicsConstant;
        }
        double getControllerAngleFromCenter(frc::Joystick *controller) {

            const double x = -controller->GetX(frc::GenericHID::kRightHand);
            const double y = -controller->GetY(frc::GenericHID::kRightHand);

            VectorDouble center(0, 1);
            VectorDouble current(x, y);
            const double dotProduct = center * current;
            const double magnitudeProduct = center.magnitude() * current.magnitude();
            const double cosineAngle = dotProduct / magnitudeProduct;
            return acos(cosineAngle);
        }
        double getControllerAbsoluteMagnitude(frc::Joystick *controller) {

            //Get the absolute values of the joystick coordinates
            double absX = abs(controller->GetX(frc::GenericHID::kRightHand));
            double absY = abs(controller->GetY(frc::GenericHID::kRightHand));

            //Return the sum of the coordinates as a knock-off magnitude
            return absX + absY;
        }
        bool getControllerAllInDeadzone(frc::Joystick *controller) {

            const double absLeftX = abs(controller->GetX(frc::GenericHID::kRightHand));
            const double absLeftY = abs(controller->GetY(frc::GenericHID::kRightHand));
            const double absZ = abs(controller->GetZ());
            const double zone = R_controllerDeadzone;

            if (absLeftX < zone && absLeftY < zone && absZ < R_twistDeadzone) {

                return true;
            }
            return false;
        }
};
