#include <math.h>
#include <cmath>

#include <frc/Joystick.h>

#include "SwerveTrain.h"
#include "VectorDouble.h"

/*
void SwerveTrain::driveController(frc::Joystick *controller) {

    const double controllerREVRotationsFromCenter = getControllerClockwiseREVRotationsFromCenter(controller);
    const double controllerMagnitude = getControllerAbsoluteMagnitude(controller);
    //TODO: Why is there a negative here?
    const double controllerTurningMagnitude = -controller->GetZ();

    if (getControllerInDeadzone(controller)) {

        assumeNearestZeroPosition();
        setDriveSpeed(0);
    }
    //If we're out of deadzone and turning is also out of its deadzone, override driving and begin turning...
    else if (abs(controllerTurningMagnitude) > R_controllerZDeadzone) {

        //To avoid inverting the drive motor direction in turning, rotate each swerve away from center
        //by a successively increasing amount: the first moves a total of 45*, the second a total of 135*,
        //the third a total of 225*, and the fourth a total of 315*. These are taken as fractional values
        //then multiplied by Nic's Constant, as assumeSwervePosition() operates on REV Rotation Values,
        //also called shorthand as Nics.
        m_frontRight->assumeSwervePosition((1.0 / 8.0) * R_nicsConstant);
        m_frontLeft->assumeSwervePosition((3.0 / 8.0) * R_nicsConstant);
        m_rearLeft->assumeSwervePosition((5.0 / 8.0) * R_nicsConstant);
        m_rearRight->assumeSwervePosition((7.0 / 8.0) * R_nicsConstant);
        setDriveSpeed(controllerTurningMagnitude * R_zionExecutionCap);
    }
    //Otherwise, simply drive normally.
    else {

        m_frontRight->assumeSwervePosition(controllerREVRotationsFromCenter);
        m_frontLeft->assumeSwervePosition(controllerREVRotationsFromCenter);
        m_rearLeft->assumeSwervePosition(controllerREVRotationsFromCenter);
        m_rearRight->assumeSwervePosition(controllerREVRotationsFromCenter);
        setDriveSpeed(controllerMagnitude * R_zionExecutionCap);
    }
}

*/


void SwerveTrain::driveController(frc::Joystick *controller) {
    double x = -controller->GetX(frc::GenericHID::kLeftHand);
    //Y seems to be inverted by default, so un-invert it...
    double y = -controller->GetY(frc::GenericHID::kLeftHand);

    double z = -controller->GetZ(); 

    forceZeroControllerXYZToDeadzone(x, y, z); 

    VectorDouble translationVector = getTranslationVector(x, y, navX->getYaw());
    
    VectorDouble frontRightRotationVector(z * cos (tangentialAngleFromRobotRadius), -z * sin (tangentialAngleFromRobotRadius) ); 
    VectorDouble frontLeftRotationVector(z * cos (tangentialAngleFromRobotRadius), z * sin (tangentialAngleFromRobotRadius) ); 
    VectorDouble rearLeftRotationVector(-z * cos (tangentialAngleFromRobotRadius), z * sin (tangentialAngleFromRobotRadius) ); 
    VectorDouble rearRightRotationVector(-z * cos (tangentialAngleFromRobotRadius), -z * sin (tangentialAngleFromRobotRadius) );
    

    VectorDouble frontRightResultVector = translationVector + frontRightRotationVector; 
    VectorDouble frontLeftResultVector = translationVector + frontLeftRotationVector;
    VectorDouble rearLeftResultVector = translationVector + rearLeftRotationVector;
    VectorDouble rearRightResultVector = translationVector + rearRightRotationVector;


    if(getControllerInDeadzone(controller)) {
         
         assumeNearestZeroPosition(); 
         setSwerveZeroPosition(); 
         resetRotationDegreeOffset(); 

    } else {
        m_frontRight->assumeSwervePosition(getVectorClockwiseREVRotationsFromCenter(frontRightResultVector));
        m_frontLeft->assumeSwervePosition(getVectorClockwiseREVRotationsFromCenter(frontLeftResultVector));
        m_rearLeft->assumeSwervePosition(getVectorClockwiseREVRotationsFromCenter(rearLeftResultVector));
        m_rearRight->assumeSwervePosition(getVectorClockwiseREVRotationsFromCenter(rearRightResultVector));
        setDriveSpeed(frontRightResultVector.magnitude() * R_zionExecutionCap);
    }

}

double SwerveTrain::getVectorClockwiseREVRotationsFromCenter(VectorDouble const &vector) {
    //TODO: Why is there a negative here?
    const double x = vector.i; 
    //Y seems to be inverted by default, so un-invert it...
    const double y = vector.j;

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


double SwerveTrain::getControllerClockwiseREVRotationsFromCenter(frc::Joystick *controller) {

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

double SwerveTrain::getControllerAngleFromCenter(frc::Joystick *controller) {

    const double x = -controller->GetX();
    const double y = -controller->GetY();

    VectorDouble center(0, 1);
    VectorDouble current(x, y);
    const double dotProduct = center * current;
    const double magnitudeProduct = center.magnitude() * current.magnitude();
    const double cosineAngle = dotProduct / magnitudeProduct;
    return acos(cosineAngle);
}
