#include <math.h>

#include <frc/XboxController.h>

#include "SwerveTrain.h"
#include "VectorDouble.h"

void SwerveTrain::driveController(frc::XboxController *controller) {

    const double controllerREVRotationsFromCenter = getControllerClockwiseREVRotationsFromCenter(controller);
    const double controllerMagnitude = getControllerAbsoluteMagnitude(controller);

    //TODO: Why is there a negative here?
    const double controllerTurningMagnitude = -controller->GetX(frc::GenericHID::kRightHand);

    if (getControllerAllInDeadzone(controller)) {
        
        assumeNearestZeroPosition();
        setDriveSpeed(0);
    }
    //If the we're out of deadzone and turning is out of deadzone, override driving and begin turning...
    else if (abs(controllerTurningMagnitude) > R_controllerDeadzone) {

        //To avoid inverting the drive motor direction in turning, rotate each swerve away from center
        //by a successively increasing amount: the first moves a total of 45*, the second a total of 135*,
        //the third a total of 225*, and the fourth a total of 315*. These are taken as fractional values
        //then multiplied by Nic's Constant, as assumeSwervePosition() operates on REV Rotation Values,
        //also called shorthand as Nics.
        m_frontRight->assumeSwervePosition((1.0 / 8.0) * R_nicsConstant);
        m_frontLeft->assumeSwervePosition((3.0 / 8.0) * R_nicsConstant);
        m_rearLeft->assumeSwervePosition((5.0 / 8.0) * R_nicsConstant);
        m_rearRight->assumeSwervePosition((7.0 / 8.0) * R_nicsConstant);
        setDriveSpeed(controllerTurningMagnitude * R_driveTrainExecutionCap);
    }
    //Otherwise, simply drive normally.
    else {
        
        m_frontRight->assumeSwervePosition(controllerREVRotationsFromCenter);
        m_frontLeft->assumeSwervePosition(controllerREVRotationsFromCenter);
        m_rearLeft->assumeSwervePosition(controllerREVRotationsFromCenter);
        m_rearRight->assumeSwervePosition(controllerREVRotationsFromCenter);
        setDriveSpeed(controllerMagnitude * R_driveTrainExecutionCap);
    }
}

double SwerveTrain::getControllerClockwiseREVRotationsFromCenter(frc::XboxController *controller) {

    //TODO: Why is there a negative here?
    const double x = -controller->GetX(frc::GenericHID::kLeftHand);
    //Y seems to be inverted by default, so un-invert it...
    const double y = -controller->GetY(frc::GenericHID::kLeftHand);

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