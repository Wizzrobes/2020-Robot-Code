/*
class HAL : public SwerveTrain

Constructors

    HAL(Arduino&, Intake&, Launcher&, Limelight&, NavX&, SwerveTrain&)
        Creates an autonomous driver with access to everything necessary
        for autonomous operation on the robot.

Public Methods

    void zionAssumeAngle(const double&)
        Rotates to the supplied angle based on the current angle of the NavX
        attached to Zion, with speed to do so found via regression.
    void zionLineupToTarget(const double&, const double&, const double&, const double&)
        Uses two distances from the wall (left and right from the front), an
        offset from a target, and the desired distance away from the target to
        drive Zion such that it will square itself with the wall, shift to align
        with the target, and then move forward or backward to achieve the
        desired distance.
    void zionShootingPositionToTrenchGrab()
        Moves laterally and rotationally from the auto shooting position
        in front of the high goal through the trench to pick up more
        Power Cells. Actuates the intake appropriately in the process.
    void zionTrenchGrabToShootingPosition()
        Same as above, but minus the intake and the exact opposite
        movements.

    enum ZionDirections
        Used by the assume functions to specify which directions to
        apply motion.

Private Methods

    double calculateAssumeAngleRotationSpeed(const double&)
        Calculates the speed at which to rotate for the assumeAngle() function
        based on how far away from the target angle we are. Uses a regression
        to do so; very similar to calculateAssumePositionSpeed() in
        SwerveModule.
    void zionAssumeDirection(const int&)
        Uses a supplied ZionDirections to set the swerves to the
        appropriate position to move in that direction. Sets no speed
        to the motors!
    void zionAssumeDistance(const double&)
        Uses the supplied distance to move that far in whatever direction
        the swerves are currently set for. As such, the usual order is a
        zionAssumeDirection followed by this. Zero speed is set once the
        distance is achieved.
    void zionTurn(const bool& = true)
        Sets the swerves to 45* positions and turns them at the global auto
        rotation speed clockwise if true, counterclockwise if false.
        Does no cleanup! Therefore, run it in a conditional and clean it up.
        TODO: make that better ;)
*/

#include <math.h>

#include "Arduino.h"
#include "Intake.h"
#include "Launcher.h"
#include "Limelight.h"
#include "NavX.h"
#include "RobotMap.h"
#include "SwerveTrain.h"
#include "VectorDouble.h"

class HAL : public SwerveTrain {

    public:
        HAL(Arduino &refArduino, Intake &refIntake, Launcher &refLauncher, Limelight &refLimelight, NavX &refNavX, SwerveTrain &refZion) {

            *arduino = refArduino;
            *intake = refIntake;
            *launcher = refLauncher;
            *limelight = refLimelight;
            *navX = refNavX;
            *zion = refZion;
        }

        void zionAssumeAngle(const double &angle) {

            const double currentAngleOff = navX->getYaw() - angle;

            //If the current angle that we're off is not within tolerance...
            if (abs(currentAngleOff) > R_zionAutoToleranceAngle) {

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
                setDriveSpeed();
            }
        }

        void zionLineupToTarget(const double &leftDistToWall, const double &rightDistToWall, const double &targetOffset, const double &targetDistance) {

            //TODO: ?
            double rotationDirection = 0;
            double averageDistanceFromWall = (leftDistToWall + rightDistToWall) / 2;
            //The distance to shoot from away from the wall in inches.
            double shotDistance = 144;

            //if and else if statements make sure the program runs in proper order-
            //rotation to be square with the wall, left-to-right lateral motion to be
            //centered with the target, and then front-to-back motion to set the proper
            //distance from the target.
            //If Zion is not plumb with the wall:
            if (abs(leftDistToWall - rightDistToWall) > R_zionAutoToleranceDistance) {

                //Rotate clockwise if right distance is less than the left,
                //counterclockwise if left distance is less than the right,
                //and set straught if equal, as this is the direction in
                //which rotation must commence to be plumb with the wall.
                if (leftDistToWall > rightDistToWall) {

                    zionTurn(true);
                }
                else if (rightDistToWall > leftDistToWall) {

                    zionTurn(false);
                }
                else {

                    zionAssumeDirection(ZionDirections::kForward);
                }
            }
            //Next, if we're plumb with the wall, but not centered:
            else if (abs(targetOffset) < R_zionAutoToleranceDistance) {

                //Move left or right based on the x-offset to be centered.
                if (targetOffset > 0) {

                    zionAssumeDirection(ZionDirections::kLeft);
                    setDriveSpeed(R_zionAutoMovementSpeedLateral);
                }
                else if (targetOffset < 0) {

                    zionAssumeDirection(ZionDirections::kRight);
                    setDriveSpeed(R_zionAutoMovementSpeedLateral);
                }
                else {

                    //If finished, set forward in preparation for next step.
                    zionAssumeDirection(ZionDirections::kForward);
                }
            }
            //Finally, if we're plumb and centered but off of target distance:
            else if (abs(averageDistanceFromWall - targetDistance) > R_zionAutoToleranceDistance) {

                if (averageDistanceFromWall > shotDistance) {

                    setDriveSpeed(-R_zionAutoMovementSpeedLateral);
                }
                else if (averageDistanceFromWall < shotDistance) {

                    setDriveSpeed(R_zionAutoMovementSpeedLateral);
                }
                else {

                    setDriveSpeed();
                }
            }
        }
        void zionShootingPositionToTrenchGrab() {
        
            //Static distances to move. Will change based on trench
            //measurements. All units in inches.
            //Movement to left from start, in inches, to line up with
            //the trench.
            double distanceLeft = 30;
            //Movement forward into the trench.
            double distanceForward = 60;

            //Rotate 90* to line up the intake to the trench. TODO: Which direction?
            zionAssumeAngle(navX->getAngle() + 90.);

            //Move left the appropriate distance.
            zionAssumeDirection(ZionDirections::kLeft);
            zionAssumeDistance(distanceLeft);
            //Same for forward.
            zionAssumeDirection(ZionDirections::kForward);
            zionAssumeDistance(distanceForward);
        }
        void zionTrenchGrabToShootingPosition() {
   
            double distanceBackward = 60;
            double distanceRight = 30;

            zionAssumeDirection(ZionDirections::kBackward);
            zionAssumeDistance(distanceBackward);
            zionAssumeDirection(ZionDirections::kRight);
            zionAssumeDistance(distanceRight);

            zionAssumeAngle(navX->getAngle() + 90.);
        }

        enum ZionDirections {

            kForward, kRight, kBackward, kLeft
        };

    private:
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
        void zionAssumeDirection(const int &directionToMove) {

            VectorDouble forward(0,-1);
            VectorDouble right(-1,0);
            VectorDouble backward(0,1);
            VectorDouble left(1,0);

            void setZionMotorsToVector(VectorDouble &vectorToSet) {

                m_frontRight->assumeSwervePosition(getClockwiseREVRotationsFromCenter(vectortoSet));
                m_frontLeft->assumeSwervePosition(getClockwiseREVRotationsFromCenter(vectorToSet));
                m_rearLeft->assumeSwervePosition(getClockwiseREVRotationsFromCenter(vectorToSet));
                m_rearRight->assumeSwervePosition(getClockwiseREVRotationsFromCenter(vectorToSet));
            }

            switch (directionToMove) {

                case kForward: setZionMotorsToVector(forward); break;
                case kRight: setZionMotorsToVector(right); break;
                case kBackward: setZionMotorsToVector(backward); break;
                case kLeft: setZionMotorsToVector(left); break;
            }
        }
        void zionAssumeDistance(const double &distanceToMove) {

            double encoderValue = m_frontRight->getDrivePosition();
            //Calculate the end goal encoder value with circumference and the
            //known amount of encoder values per rotation.
            double encoderValueGoal = encoderValue + ((distanceToMove / circumferenceWheel) * R_kuhnsConstant);
            //Move until we're there and stop.
            if ((encoderValueGoal - m_frontRight->getDrivePosition()) > 0) {

                setDriveSpeed(R_zionAutoMovementSpeedLateral);
            }
            else {

                setDriveSpeed();
            }
        }
        //TODO: Inline function documentation
        void zionTurn(const bool &direction = true) {

            double sqt = sqrt(2) / 2;
            double spd = R_zionAutoMovementSpeedRotational;

            VectorDouble vectorFrontRight(spd * sqt, spd * sqt);
            VectorDouble vectorFrontLeft(spd * sqt, -spd * sqt);
            VectorDouble vectorRearLeft(-spd * sqt, -spd * sqt);
            VectorDouble vectorRearRight(-spd * sqt, spd * sqt);

            m_frontRight->assumeSwervePosition(getClockwiseREVRotationsFromCenter(vectorFrontRight));
            m_frontLeft->assumeSwervePosition(getClockwiseREVRotationsFromCenter(vectorFrontLeft));
            m_rearLeft->assumeSwervePosition(getClockwiseREVRotationsFromCenter(vectorRearLeft));
            m_rearRight->assumeSwervePosition(getClockwiseREVRotationsFromCenter(vectorRearRight));

            setDriveSpeed(direction ? spd : -spd);
        }

        Arduino *arduino;
        Intake *intake;
        Launcher *launcher;
        Limelight *limelight;
        NavX *navX;
        SwerveTrain *zion;

        double circumferenceWheel = 4 * M_PI;
};
