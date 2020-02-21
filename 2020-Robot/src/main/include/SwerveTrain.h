/*
class SwerveTrain

    Allows higher-level control of four SwerveModules as a drivetrain, and
        provides many private controller functions for manipulating it.

Constructors

    SwerveTrain(SwerveModule&, SwerveModule&, SwerveModule&, SwerveModule&,
        NavX&):
        Creates a swerve train with the swerve modules on the front right,
        front left, back left, and back right positions, and takes a NavX
        for use in calculating rotational vectors.

Public Methods

    void setDriveSpeed(const double&): Sets a speed to all driving motors on
        the train.
    void setSwerveSpeed(const double&): Sets a speed to all swerve motors on
        the train.
    void setZeroPosition(const bool&): Gets the current encoder values
        of the swerve motors and stores them as privates of the class. These
        are the values the swerve motors return to when invoking
        assumeSwerveZeroPosition().
        If the passed bool is true, publishes the stored data to the
        SmartDashboard. This is currently used for returning to and maintaining
        "straight".
    void assumeZeroPosition(): Drives the swerves to return to their
        zero position.
    void assumeNearestZeroPosition(): Drives the swerves to the
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
    void assumeAngle(const double&): Rotates to the supplied angle based
        on the current angle of the NavX attached to Zion. Finding this
        angle will likely require using the gyro, as it is relative.
    void driveController(): Fully drives the swerve train on the supplied
        controller.
    void zeroController(): Allows use of a controller through
        a mapped button which is held down in correspondence to a motor
        to slowly override its zero from that controller's joystick
        value. This allows manual adjustment from an enabled state in case of
        either drift or error. CURRENTLY WRITTEN FOR A JOYSTICK, WILL LIKELY
        NEED TO CHANGE.

Private Methods

    double getClockwiseREVRotationsFromCenter(frc::Joystick*):
        Discernes how many clockwise REV rotations from center the current
        location of the joystick is using vector trigonometry and properties.
        See https://en.wikipedia.org/wiki/Dot_product#Geometric_definition
    double getClockwiseREVRotationsFromCenter(const VectorDouble&):
        Same as above, but accepts a vector outright instead of stripping
        one from the supplied controller.
    double getStandardDegreeAngleFromCenter(const double&, const double&): Same
        as above, but returns the result as a degree measure in standard
        position.
    double getLargestMagnitudeValue(const double&, const double&, const double&, const double&):
        Returns the largest of the four values passed to the function.
    VectorDouble getTranslationVector(const double&, const double&, double):
        Calculates the translation vector to be used in total swerve movement
        calculation by the control function. See the function itself for more.
    double calculateAssumeAngleRotationSpeed(const double&): Calculates the
        speed at which to rotate for the assumeAngle() function based on how
        far away from the target angle we are. Uses a regression to do so, very
        similar to calculateAssumePositionSpeed() in SwerveModule.
    double getControllerAbsoluteMagnitude(frc::Joystick*): Gets the
        unsigned velocity of the control stick using only absolute value.
    bool getControllerInDeadzone(frc::Joystick*): If all axis of the
        controller are within their RobotMap deadzone variables for
        playerOne's controller, returns true; otherwise, returns false.
    void forceControllerXYZToZeroInDeadzone(const int&, const int&, const int&):
        If any of the passed X, Y, or Z values fall outside of their global
        deadzone, they will be set to 0. Otherwise, they are untouched.
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

        void setDriveSpeed(const double &driveSpeed) {

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

        void setZeroPosition(const bool &verbose = false) {

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

        void assumeAngle(const double &angle) {

            const double currentAngleOff = navX->getYaw() - angle;

            //If the current angle that we're off is not within tolerance...
            if (abs(currentAngleOff) > 0/*TOLERANCE*/) {

                //Set the wheels to their diagonal positions (at incremental 45*
                //angles found with radians converted into Nics) and turn
                //with the speed calculated via regression...
                m_frontRight->assumeSwervePosition((1.0 / 8.0) * R_nicsConstant);
                m_frontLeft->assumeSwervePosition((3.0 / 8.0) * R_nicsConstant);
                m_rearLeft->assumeSwervePosition((5.0 / 8.0) * R_nicsConstant);
                m_rearRight->assumeSwervePosition((7.0 / 8.0) * R_nicsConstant);
                setDriveSpeed(calculateAssumeAngleRotatationSpeed(currentAngleOff));
            }
            //Otherwise, go to the closest zero and stop moving.
            else {

                assumeNearestZeroPosition();
                setDriveSpeed(0);
            }
        }

        void driveController(frc::Joystick *controller);
        void zeroController(frc::Joystick *controller);
        void getTrenchRunPowerCells();
        void returnToShootingPosition();
        void lineupShot();
        void runZionAutonomous();
        void moveToTarget();
        

    private:
        SwerveModule *m_frontRight;
        SwerveModule *m_frontLeft;
        SwerveModule *m_rearLeft;
        SwerveModule *m_rearRight;
        NavX *navX;

        void driveAutonomous(VectorDouble movement, double speed);
        void turnAutonomous(double speed);
        void stopDriving();
        void shootAuto();
        double getClockwiseREVRotationsFromCenter(frc::Joystick *controller);
        double getClockwiseREVRotationsFromCenter(const VectorDouble &vector);
        double getStandardDegreeAngleFromCenter(const double &x, const double &y);
        double getLargestMagnitudeValue(const double &frVal, const double &flVal, const double &rlVal, const double &rrVal) {

            return std::max(std::max(frVal, flVal), std::max(rrVal, rlVal));
        }
        VectorDouble getTranslationVector(const double &x, const double &y, double angleGyro);

        double calculateAssumeAngleRotatationSpeed(const double &currentAngleOff) {

            //TODO: Document/improve regression
            double speed = pow(((2.469135802 * pow(10, -6)) * currentAngleOff), 3) + (0.0061111111 * currentAngleOff);

            if (speed > 1) {

                speed = 1;
            }
            else if (speed < -1) {

                speed = -1;
            }
            else {

                return speed;
            }
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
            const double zone = R_controllerDeadzone;

            if (absX < zone && absY < zone && absZ < zone) {

                return true;
            }
            return false;
        }
        void forceControllerXYZToZeroInDeadzone(double &x, double &y, double &z) {

            double absX = abs(x);
            double absY = abs(y);
            double absZ = abs(z);

            if (absX < R_controllerDeadzone) {x = 0;}
            if (absY < R_controllerDeadzone) {y = 0;}
            if (absZ < R_controllerZDeadzone) {z = 0;}

        }
};
