#include <frc/XboxController.h>

#include "SwerveTrain.h"

void SwerveTrain::driveController(frc::XboxController *controller) {

    const double controllerREVRotationsFromCenter = getControllerREVRotationsFromCenter(controller);
    //const double controllerMagnitude = getControllerAbsoluteMagnitude(controller);

    //If the control stick is within deadzone, return to the zero position
    if (abs(controller->GetX(frc::GenericHID::kLeftHand)) < R_playerOneControllerDeadzone && abs(controller->GetY(frc::GenericHID::kLeftHand)) < R_playerOneControllerDeadzone) {
        
        assumeSwerveNearestZeroPosition();
    }
    else {
        
        m_frontRight->assumeSwervePosition(controllerREVRotationsFromCenter);  
    }
}
