/*
class HAL

Allows low-level automatic control of the robot as a secondary driver. The
    whole class is really just a conglomarate wrapper for auto functionality.
    All that each function does at the end of the day is write values to the
    motors - just like a real driver, to preserve full compatibility with
    TeleOp. As such, cancellation is possible either by not calling the
    function any longer or by simply overwriting motor values before they
    are set at the end of the loop. Each function returns true when success
    is fulfilled, so an order of autonomous as follows:
    if (taskOne) {
        if (taskTwo) {
            if (taskThree) {
                ...
            }
        }
    }
    will complete tasks one, two, and three in that order, waiting for success
    from each before continuing, assuming that each if also has a global
    control in it if being run in a loop. See zionShootingPositionToTrenchGrab
    for an example of this.

Constructors

    HAL(Intake&, Launcher&, Limelight&, NavX&, SwerveTrain&)
        Creates an autonomous driver with access to everything necessary
        for autonomous operation on the robot.

Public Methods

    bool zionAssumeRotationDegrees(const double&)
        Rotates the desired number of degrees using the NavX sensor. Does
        so at a constant global speed; could likely be regressed similarly
        to the swerve modules. Returns to zero position when done.
    bool zionAssumeDirection(const int&)
        Uses a supplied ZionDirections to set the swerves to the
        appropriate position to move in that direction. Sets no speed
        to the motors! Thus, the normal order is direction, distance, then
        back to original direction. This system allows precise movement in
        setting the swerves to one position and then moving back and forth
        between them without swerve wobble, such as when lining up laterally
        with the high goal.
    bool zionAssumeDistance(const double&)
        Uses the supplied distance to move that far in whatever direction
        the swerves are currently set for. As such, the usual order is a
        zionAssumeDirection followed by this. Zero speed is set once the
        distance is achieved; distance measured by the circumference of a
        wheel.
    void zionShootingPositionToTrenchGrab()
        Moves laterally and rotationally from the auto shooting position
        in front of the high goal through the trench to pick up more
        Power Cells. Actuates the intake appropriately in the process.
    void zionTrenchGrabToShootingPosition()
        Same as above, but minus the intake and the exact opposite
        movements, with more launching.

    enum ZionDirections
        Used by the assume functions to specify which directions to
        apply motion.

Private Methods

    void setZionMotorsToVector(const VectorDouble &)
        Sets the swerve positions on Zion to the angle of the passed vector
        inscribed in standard position. Used to assume directions by vector.
        This makes for easy, accurate, and actually sensible positioning, as
        the vectors to move in cardinal directions are just the signed
        cartesian axes.
*/

#include <math.h>

#include "Intake.h"
#include "Launcher.h"
#include "Limelight.h"
#include "NavX.h"
#include "RobotMap.h"
#include "SwerveTrain.h"
#include "VectorDouble.h"

class Hal {

    public:
        Hal(Intake &refIntake, Launcher &refLauncher, Limelight &refLimelight, NavX &refNavX, SwerveTrain &refZion) {

            m_intake = &refIntake;
            m_launcher = &refLauncher;
            m_limelight = &refLimelight;
            m_navX = &refNavX;
            m_zion = &refZion;

            m_utilityVarsSet = true;
            m_utilityVarOne = 0;
            m_utilityVarTwo = 0;
        }

        bool zionAssumeRotationDegrees(const double &degreesToRotate) {

            //At the first iteration, set the starting and goal angles to
            //memory for comparison once operating...
            if (!m_utilityVarsSet) {

                m_utilityVarOne = m_navX->getAngle();
                m_utilityVarTwo = m_utilityVarOne + degreesToRotate;
                m_utilityVarsSet = true;
            }

            //Set the wheels to their diagonal positions (at incremental 45*
            //angles found with radians converted into Nics)...
            m_zion->m_frontRight->assumeSwervePosition((1.0 / 8.0) * R_nicsConstant);
            m_zion->m_frontLeft->assumeSwervePosition((3.0 / 8.0) * R_nicsConstant);
            m_zion->m_rearLeft->assumeSwervePosition((5.0 / 8.0) * R_nicsConstant);
            m_zion->m_rearRight->assumeSwervePosition((7.0 / 8.0) * R_nicsConstant);

            //If we're not within tolerance for meeting the goal angle...
            if (abs(m_utilityVarTwo - m_navX->getAngle()) > R_zionAutoToleranceAngle) {

                //If rotation needs to be clockwise (goal is greater than init)
                //set the turning speed to be positive, otherise, set it
                //negative...
                m_zion->setDriveSpeed(m_utilityVarTwo > m_utilityVarOne ? R_zionAutoMovementSpeedLateral : -R_zionAutoMovementSpeedLateral);
            }
            //If we were within tolerance that iteration...
            else {

                //Stop moving, clean up, and return true.
                m_zion->setDriveSpeed();
                m_zion->assumeNearestZeroPosition();
                m_utilityVarsSet = false;
                m_utilityVarOne = 0;
                m_utilityVarTwo = 0;
                return true;
            }
            //If we made it to here, we didn't succeed, so return false for
            //another go at it.
            return false;
        }
        bool zionAssumeDirection(const int &directionToMove) {

            //TODO: Why inverted?
            VectorDouble forward(0,-1);
            VectorDouble right(-1,0);
            VectorDouble backward(0,1);
            VectorDouble left(1,0);

            switch (directionToMove) {

                case kForward: setZionMotorsToVector(forward); break;
                case kRight: setZionMotorsToVector(right); break;
                case kBackward: setZionMotorsToVector(backward); break;
                case kLeft: setZionMotorsToVector(left); break;
            }
            //If all of the swerve speeds are exceedingly close to zero, it
            //means that all of the motors are done positioning, so return
            //true; otherwise, return false for another go at it.
            return m_zion->m_frontRight->getSwerveSpeed() < .05 && m_zion->m_frontLeft->getSwerveSpeed() < .05 && m_zion->m_rearLeft->getSwerveSpeed() < .05 && m_zion->m_rearRight->getSwerveSpeed() < .05 ? true : false; 
        }
        bool zionAssumeDistance(const double &distanceToMove) {

            //At the first iteration, set the starting and goal values to
            //memory for comparison once operating (since we're translating
            //in a lateral direction, we only have to care about one encoder
            //value)...
            if (!m_utilityVarsSet) {

                m_utilityVarOne = m_zion->m_frontRight->getDrivePosition();
                //Calculate the end goal encoder value with circumference and the
                //known amount of encoder values per rotation.
                m_utilityVarTwo = m_utilityVarOne + ((distanceToMove / m_circumferenceWheel) * R_kuhnsConstant);
                m_utilityVarsSet = true;
            }

            //If we're not in tolerance for meeting the goal value...
            if (m_utilityVarTwo - m_zion->m_frontRight->getDrivePosition()) {

                m_zion->setDriveSpeed(R_zionAutoMovementSpeedLateral);
            }
            //If we were...
            else {

                //Stop moving, clean up, and return true.
                m_zion->setDriveSpeed();
                m_utilityVarsSet = false;
                m_utilityVarOne = 0;
                m_utilityVarTwo = 0;
                return true;
            }
            //If we made it to here, we didn't succeed, so return false for
            //another go at it.
            return false;
        }
        bool zionShootingPositionToTrenchGrab() {
        
            //Static distances to move. Will change based on trench
            //measurements. All units in inches.
            //Movement to left from start, in inches, to line up with
            //the trench.
            double distanceLeft = 30;
            //Movement forward into the trench.
            double distanceForward = 60;

            //Perform each step of the process wih a utilityVar so each
            //step only occurs once and in order. Evaluate the utilityVar
            //first so that the functions only run when necessary and as order
            //necessitates, as && is a short-circuiting operator. Thus, if it
            //is not a function's turn to run, it does not run, and if it is,
            //it is the only one that runs.
            //Rotate 90* to line up the intake to the trench. TODO: Which direction?
            if (m_utilityVarOne == 0 && zionAssumeRotationDegrees(90)) {

                ++m_utilityVarOne;
            }
            //Move left the appropriate distance.
            if (m_utilityVarOne == 1 && zionAssumeDirection(ZionDirections::kLeft)) {

                ++m_utilityVarOne;
            }
            if (m_utilityVarOne == 2 && zionAssumeDistance(distanceLeft)) {

                ++m_utilityVarOne;
            }
            //Same for forward.
            if (m_utilityVarOne == 3 && zionAssumeDirection(ZionDirections::kForward)) {

                ++m_utilityVarOne;
            }
            if (m_utilityVarOne == 4 && zionAssumeDistance(distanceForward)) {

                //This is the last step, so if it was successful, return true
                //and clean up.
                m_utilityVarOne = 0;
                return true;
            }
            //If we've made it here, some step of the process failed, so return
            //false to give it another go.
            return false;
        }
        //TODO: If/WHEN the other one works, rewrite this one similarly
        void zionTrenchGrabToShootingPosition() {
   
            double distanceBackward = 60;
            double distanceRight = 30;

            zionAssumeDirection(ZionDirections::kBackward);
            zionAssumeDistance(distanceBackward);
            zionAssumeDirection(ZionDirections::kRight);
            zionAssumeDistance(distanceRight);

            zionAssumeRotationDegrees(m_navX->getAngle() + 90.);
        }

        enum ZionDirections {

            kForward, kRight, kBackward, kLeft
        };

    private:
        void setZionMotorsToVector(const VectorDouble &vectorToSet) {

            m_zion->m_frontRight->assumeSwervePosition(m_zion->getClockwiseREVRotationsFromCenter(vectorToSet));
            m_zion->m_frontLeft->assumeSwervePosition(m_zion->getClockwiseREVRotationsFromCenter(vectorToSet));
            m_zion->m_rearLeft->assumeSwervePosition(m_zion->getClockwiseREVRotationsFromCenter(vectorToSet));
            m_zion->m_rearRight->assumeSwervePosition(m_zion->getClockwiseREVRotationsFromCenter(vectorToSet));
        }

        Intake *m_intake;
        Launcher *m_launcher;
        Limelight *m_limelight;
        NavX *m_navX;
        SwerveTrain *m_zion;

        double m_circumferenceWheel = 4 * M_PI;

        //These are used by the function for values which need to persist
        //across multiple operating calls of the function. What they are is
        //defined in each function. Be safe with them - they're global to Hal.
        bool m_utilityVarsSet;
        double m_utilityVarOne;
        double m_utilityVarTwo;
};

//TODO: Rewrite for new Limelight distance implement
/*
    void zionLineupToTarget(const double&, const double&, const double&, const double&)
        Uses two distances from the wall (left and right from the front), an
        offset from a target, and the desired distance away from the target to
        drive Zion such that it will square itself with the wall, shift to align
        with the target, and then move forward or backward to achieve the
        desired distance.

        void whileLineupToTarget(const double &targetDistance) {

            //while statements make sure the program runs in proper order-
            //rotation to be square with the wall, left-to-right lateral motion to be
            //centered with the target, and then front-to-back motion to set the proper
            //distance from the target.
            //If Zion is not plumb with the wall:
            while (!arduino->getSonarNormal()) {

                //Rotate clockwise if right distance is less than the left,
                //counterclockwise if left distance is less than the right,
                //and set straught if equal, as this is the direction in
                //which rotation must commence to be plumb with the wall.
                if (arduino->getSonarSkew(Arduino::SonarSkews::kLeft)) {

                    ifZionTurn(true);
                }
                else if (arduino->getSonarSkew(Arduino::SonarSkews::kRight)) {

                    ifZionTurn(false);
                }
                else {

                    whileZionAssumeDirection(ZionDirections::kForward);
                }
            }
            //Next, if we're plumb with the wall, but not centered:
            while (abs(limelight->horizontalOffset()) > R_zionAutoToleranceHorizontalOffset) {

                //Move left or right based on the x-offset to be centered.
                if (limelight->horizontalOffset() > 0) {

                    whileZionAssumeDirection(ZionDirections::kLeft);
                    zion->setDriveSpeed(R_zionAutoMovementSpeedLateral);
                }
                else if (limelight->horizontalOffset() < 0) {

                    whileZionAssumeDirection(ZionDirections::kRight);
                    zion->setDriveSpeed(R_zionAutoMovementSpeedLateral);
                }
                else {

                    //If finished, set forward in preparation for next step.
                    whileZionAssumeDirection(ZionDirections::kForward);
                }
            }
            //Finally, if we're plumb and centered but off of target distance:
            while (arduino->getSonarTooCloseTarget() || arduino->getSonarTooFarTarget()) {

                if (arduino->getSonarTooCloseTarget()) {

                    zion->setDriveSpeed(-R_zionAutoMovementSpeedLateral);
                }
                else if (arduino->getSonarTooFarTarget()) {

                    zion->setDriveSpeed(R_zionAutoMovementSpeedLateral);
                }
            }
            //If we're here, we're done! Stop :)
            zion->setDriveSpeed();
        }
*/
