/*
class SwerveTrain

    Allows higher-level control of four SwerveModules as a drivetrain, and
        provides many private controller functions for manipulating it.

Constructors

    SwerveTrain(SwerveModule&, SwerveModule&, SwerveModule&, SwerveModule&, NavX&)
        Creates a swerve train with the swerve modules on the front right,
        front left, back left, and back right positions, and takes a NavX
        for use in calculating rotational vectors.

Public Methods

    void setDriveSpeed(const double&)
        Sets a speed to the driving motors on the train.
    void setSwerveSpeed(const double&)
        Sets a speed to all swerve motors on the train.
    void setSwerveBrake(const bool&)
        If true, sets the swerves to brake mode (as defaultly constructed),
        if false, sets them to coast. Persists across calls. This is used to
        "unlock and lock" all of the swerve wheels for easy manual zeroing,
        instead of fighting the wheel brake as defaultly constructed.
    void setZeroPosition(const bool& = false)
        Gets the current encoder values of the swerve motors and stores them
        as privates of the class. These are the values the swerve motors return
        to when invoking assumeSwerveZeroPosition().
        If the passed bool is true, publishes the stored data to the
        SmartDashboard. This is currently used for returning to and maintaining
        "straight".
    void assumeZeroPosition()
        Drives the swerves to return to their zero position.
    void assumeNearestZeroPosition()
        Drives the swerves to the nearest Nic's Constant multiple of its zero
        value, CW or CCW. In doing so, really only drives the swerve to either
        0 or the value of one Nic's Constant. Since the pathfinding function
        thinks clockwise, this ends up driving it to zero the fastest
        way possible, making use of getSwervePositionSingleRotation() as
        the driver function to make this possible. assumeSwerveZeroPosition()
        cannot make this optimization, and simply goes to whatever the zero
        value is. Useful for low-level things.
    void publishSwervePositions()
        Puts the current swerve encoder positions to the SmartDashboard.
    void driveController()
        Fully drives the swerve train on the supplied controller.
    void zeroController()
        Allows use of a controller through a mapped button which is held down
        in correspondence to a motor to slowly override its zero from that
        controller's joystick value. This allows manual adjustment from an
        enabled state in case of either drift or error.
        TODO: CURRENTLY WRITTEN FOR A JOYSTICK, WILL NEED TO CHANGE.
    double getClockwiseREVRotationsFromCenter(frc::Joystick*)
        Discernes how many clockwise REV rotations from center the current
        location of the joystick is using vector trigonometry and properties.
        See https://en.wikipedia.org/wiki/Dot_product#Geometric_definition

Private Methods

    double getClockwiseREVRotationsFromCenter(const VectorDouble&)
        Same as above, but accepts a vector outright instead of stripping
        one from the supplied controller.
    double getStandardDegreeAngleFromCenter(const double&, const double&)
        Same as above, but returns the result as a degree measure in standard
        position.
    double getLargestMagnitudeValue(const double&, const double&, const double&, const double&)
        Returns the largest of the four values passed to the function.
    double getControllerAbsoluteMagnitude(frc::Joystick*)
        Gets the unsigned velocity of the control stick using only absolute
        value.
    bool getControllerInDeadzone(frc::Joystick*)
        If all axis of the controller are within their RobotMap deadzone
        variables for playerOne's controller, returns true; otherwise, returns
        false.
    void forceControllerXYZToZeroInDeadzone(const int&, const int&, const int&)
        If any of the passed X, Y, or Z values fall outside of their global
        deadzone, they will be set to 0. Otherwise, they are untouched.
    void optimizeControllerXYToZ(const double&, const double&, double &)
        Scales the value of Z with a propotion constant to the magnitude of
        X and Y. Makes rotation harder to incude as speed increases, which
        makes strafing with a joystick much more reliable.
*/

#pragma once

#include <math.h>

#include <frc/Joystick.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "rev/CANSparkMax.h"

#include "NavX.h"
#include "SwerveModule.h"
#include "VectorDouble.h"

class SwerveTrain {

    public:
        SwerveTrain(SwerveModule &frontRightModule, SwerveModule &frontLeftModule, SwerveModule &rearLeftModule, SwerveModule &rearRightModule, NavX &navXToSet) {

            m_frontRight = &frontRightModule;
            m_frontLeft = &frontLeftModule;
            m_rearLeft = &rearLeftModule;
            m_rearRight = &rearRightModule;
            navX = &navXToSet;
        }

        void setDriveSpeed(const double &driveSpeed = 0) {

            m_frontRight->setDriveSpeed(driveSpeed);
            m_frontLeft->setDriveSpeed(driveSpeed);
            m_rearLeft->setDriveSpeed(driveSpeed);
            m_rearRight->setDriveSpeed(driveSpeed);
        }
        void setSwerveSpeed(const double &swerveSpeed) {

            m_frontRight->setSwerveSpeed(swerveSpeed);
            m_frontLeft->setSwerveSpeed(swerveSpeed);
            m_rearLeft->setSwerveSpeed(swerveSpeed);
            m_rearRight->setSwerveSpeed(swerveSpeed);
        }
        void setSwerveBrake(const bool &brake) {

            m_frontRight->setSwerveBrake(brake);
            m_frontLeft->setSwerveBrake(brake);
            m_rearLeft->setSwerveBrake(brake);
            m_rearRight->setSwerveBrake(brake);
        }

        void setZeroPosition(const bool &verbose = false) {

            m_frontRight->setZeroPosition();
            m_frontLeft->setZeroPosition();
            m_rearLeft->setZeroPosition();
            m_rearRight->setZeroPosition();

            if (verbose) {

                frc::SmartDashboard::PutNumber("Zion::Swerve::0PosFR", m_frontRight->getSwerveZeroPosition());
                frc::SmartDashboard::PutNumber("Zion::Swerve::0PosFL", m_frontLeft->getSwerveZeroPosition());
                frc::SmartDashboard::PutNumber("Zion::Swerve::0PosRL", m_rearLeft->getSwerveZeroPosition());
                frc::SmartDashboard::PutNumber("Zion::Swerve::0PosRR", m_rearRight->getSwerveZeroPosition());
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

            frc::SmartDashboard::PutNumber("Zion::Swerve::PosFR", m_frontRight->getSwervePosition());
            frc::SmartDashboard::PutNumber("Zion::Swerve::PosFL", m_frontLeft->getSwervePosition());
            frc::SmartDashboard::PutNumber("Zion::Swerve::PosRL", m_rearLeft->getSwervePosition());
            frc::SmartDashboard::PutNumber("Zion::Swerve::PosRR", m_rearRight->getSwervePosition());
        }

        void driveController(frc::Joystick *controller);
        void driveControllerPrecision(frc::Joystick *controller); 
        void zeroController(frc::Joystick *controller);

    private:

        double getClockwiseREVRotationsFromCenter(frc::Joystick *controller);
    public:

        //This is very useful in accurate auto positioning, so it is
        //overriden public, specifically for Hal pass use at a low level.
        double getClockwiseREVRotationsFromCenter(const VectorDouble &vector);
    private:

        double getStandardDegreeAngleFromCenter(const double &x, const double &y);
        double getLargestMagnitudeValue(const double &frVal, const double &flVal, const double &rlVal, const double &rrVal) {

            return std::max(std::max(frVal, flVal), std::max(rrVal, rlVal));
        }

        double getControllerAbsoluteMagnitude(frc::Joystick *controller) {

            //Get the absolute values of the joystick coordinates
            double absX = abs(controller->GetX());
            double absY = abs(controller->GetY());

            //Return the sum of the coordinates as a knock-off magnitude
            return absX + absY;
        }
        bool getControllerInDeadzone(frc::Joystick *controller) {

            const double absX = abs(controller->GetX());
            const double absY = abs(controller->GetY());
            const double absZ = abs(controller->GetZ());
            const double zone = R_deadzoneController;

            if (absX < zone && absY < zone && absZ < zone) {

                return true;
            }
            return false;
        }
        void forceControllerXYZToZeroInDeadzone(double &x, double &y, double &z) {

            double absX = abs(x);
            double absY = abs(y);
            double absZ = abs(z);

            if (absX < R_deadzoneController) {x = 0;}
            if (absY < R_deadzoneController) {y = 0;}
            if (absZ < R_deadzoneController) {z = 0;}
        }
        //TODO: Inline function documentation
        void optimizeControllerXYToZ(const double &x, const double &y, double &z) {

            double magnitudeXY = sqrt(x * x + y * y);
            double absZ = abs(z);
            double deadzoneAdjustmentZ = R_deadzoneControllerZ + .3 * magnitudeXY * R_deadzoneControllerZ;

            if (z > deadzoneAdjustmentZ) {

                z -= (deadzoneAdjustmentZ - R_deadzoneController);
            }
            else if (z < -deadzoneAdjustmentZ) {

                z += (deadzoneAdjustmentZ - R_deadzoneController);
            }
            if (absZ < deadzoneAdjustmentZ) {

                z = 0;
            }
        }

    //Allow the peices of the SwerveTrain to be public for convenient
    //low-level access when needed. SwerveTrain is a great container.
    //This is primarily used for Hal, the auto driver, so he can set low-level
    //module commands through one passed SwerveTrain.
    public:
        SwerveModule *m_frontRight;
        SwerveModule *m_frontLeft;
        SwerveModule *m_rearLeft;
        SwerveModule *m_rearRight;
        NavX *navX;
};
